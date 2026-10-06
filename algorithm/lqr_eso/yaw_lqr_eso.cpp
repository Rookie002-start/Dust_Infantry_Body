/**
 * @file yaw_lqr_eso.cpp
 * @author Rookie002-start
 * @brief 单轴 LQR + 三阶 ESO 控制器实现
 * @version 0.1
 * @date 2026-10-03
 *
 * @copyright Copyright (c) 2026
 */
#pragma message "Compiling Algorithm/LqrEso"

#include "yaw_lqr_eso.hpp"

#include <math.h>

namespace alg::lqr_eso {

namespace {
constexpr float kMinJ      = 1.0e-6f;
constexpr float kMinDt     = 1.0e-5f;
constexpr float kMaxDt     = 5.0e-2f;
constexpr float kDefaultDt = 1.0e-3f;

inline float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/// 对称限幅
/// limit ≤ 0 视为不限幅
inline float ClampAbs(float v, float limit)
{
    return limit > 0.0f ? Clamp(v, -limit, limit) : v;
}

/// 死区：|v| ≤ db 输出 0，否则扣掉死区宽度
inline float Deadband(float v, float db)
{
    if (db <= 0.0f) {
        return v;
    }
    if (fabsf(v) <= db) {
        return 0.0f;
    }
    return v > 0.0f ? v - db : v + db;
}

/// dt 越界（过小/过大）时回落到默认 1ms，防止积分/观测器发散
inline float SanitizeDt(float dt)
{
    return (dt < kMinDt || dt > kMaxDt) ? kDefaultDt : dt;
}

/// 门限判定
/// limit ≤ 0 视为不限制
inline bool GatePass(float v, float limit)
{
    return limit <= 0.0f || fabsf(v) <= limit;
}
} // namespace

/// 载入配置并复位全部状态
void LqrEso::Init(const Config& cfg)
{
    cfg_ = cfg;
    Reset(0.0f, 0.0f);
}

/// 清积分/偏置/滤波与上一拍力矩，并用当前测量初始化 ESO 观测量
void LqrEso::Reset(float theta, float omega)
{
    eso_.z1 = theta;
    eso_.z2 = omega;
    eso_.z3 = 0.0f;

    intTheta_    = 0.0f;
    tauBias_     = 0.0f;
    tauMeasFilt_ = 0.0f;
    tauCmd_      = 0.0f;
    ready_       = false;

    UpdateEsoGains();
}

/// 按 ω0 整定三阶 ESO 增益：b0 = 1/J，β1/β2/β3 = 3ω0 / 3ω0² / ω0³
void LqrEso::UpdateEsoGains()
{
    const bool ok = cfg_.esoEnable && cfg_.j > kMinJ && cfg_.esoBandwidth > 0.0f;
    const float w0 = ok ? cfg_.esoBandwidth : 0.0f;

    eso_.b0 = ok ? 1.0f / cfg_.j : 0.0f;
    eso_.w0 = w0;
    eso_.beta1 = 3.0f * w0;
    eso_.beta2 = 3.0f * w0 * w0;
    eso_.beta3 = w0 * w0 * w0;
}

/// 三阶 ESO 一步观测：e = θ - z1，z1/z2/z3 分别跟踪角度、角速度、总扰动
void LqrEso::UpdateEso(float thetaMeas, float u, float dt)
{
    if (eso_.b0 <= 0.0f || eso_.w0 <= 0.0f) {
        return;
    }

    const float damping = cfg_.b / cfg_.j;
    const float e = thetaMeas - eso_.z1;
    const float z1Dot = eso_.z2 + eso_.beta1 * e;
    const float z2Dot =
        -damping * eso_.z2 + eso_.b0 * u + eso_.z3 + eso_.beta2 * e;
    const float z3Dot = eso_.beta3 * e;

    eso_.z1 += dt * z1Dot;
    eso_.z2 += dt * z2Dot;
    eso_.z3 += dt * z3Dot;
}

/// 由扰动估计折算补偿力矩 τ = -gain·z3/b0，并限幅
float LqrEso::EsoComp() const
{
    if (cfg_.esoCompGain == 0.0f || eso_.b0 <= kMinJ) {
        return 0.0f;
    }
    return ClampAbs(-cfg_.esoCompGain * eso_.z3 / eso_.b0, cfg_.esoCompLimit);
}

/// 库仑摩擦前馈：τc·tanh(ω_ref/smooth)，用 tanh 平滑过零
float LqrEso::CoulombComp(float omegaRef) const
{
    if (cfg_.tauCoulomb == 0.0f || cfg_.coulombSmooth <= 1.0e-6f) {
        return 0.0f;
    }
    return cfg_.tauCoulomb * tanhf(omegaRef / cfg_.coulombSmooth);
}

/// 输出文档同款状态空间：A=[[0,1],[0,-B/J]]，B=[0,1/J]ᵀ，C=[1,0]（输入按 [τ,d]）
bool LqrEso::GetStateSpace(const Config& cfg, StateSpace& model)
{
    if (cfg.j <= kMinJ) {
        return false;
    }

    const float invJ = 1.0f / cfg.j;

    model.a[0][0] = 0.0f;
    model.a[0][1] = 1.0f;
    model.a[1][0] = 0.0f;
    model.a[1][1] = -cfg.b * invJ;

    model.b[0][0] = 0.0f;
    model.b[0][1] = 0.0f;
    model.b[1][0] = invJ;
    model.b[1][1] = invJ;

    model.c[0] = 1.0f;
    model.c[1] = 0.0f;
    return true;
}

/// 主入口：ESO 观测 → 前馈+LQR → ESO/偏置补偿 → 软硬限幅 → 斜率限制，返回力矩指令
Output LqrEso::Calc(const Feedback& fb, const Reference& ref, float dt)
{
    Output out {};
    dt = SanitizeDt(dt);

    if (!fb.valid) {
        Reset(fb.theta, fb.omega);
        return out;
    }

    // 首个有效周期只同步状态，输出 0 力矩
    if (!ready_) {
        ready_       = true;
        tauMeasFilt_ = fb.tauMeas;
        eso_.z1      = fb.theta;
        eso_.z2      = fb.omega;
        eso_.z3      = 0.0f;
        out.thetaRef = ref.theta;
        out.omegaRef = ref.omega;
        out.alphaRef = ref.alpha;
        return out;
    }

    // 实测力矩低通
    const float alpha = Clamp(cfg_.tauMeasLpf, 0.0f, 1.0f);
    tauMeasFilt_ += alpha * (fb.tauMeas - tauMeasFilt_);

    // ESO 观测（用上一周期指令力矩作输入）
    UpdateEso(fb.theta, tauCmd_, dt);

    const float eTheta = Deadband(fb.theta - ref.theta, cfg_.deadband);
    const float eOmega = fb.omega - ref.omega;

    intTheta_ = ClampAbs(intTheta_ + eTheta * dt, cfg_.intLimit);

    // 前馈 + LQR 反馈
    const float tauFf  = cfg_.j * ref.alpha + cfg_.b * ref.omega +
                         CoulombComp(ref.omega);
    const float tauI    = -cfg_.kI * intTheta_;
    const float tauLqr = tauFf - cfg_.kTheta * eTheta - cfg_.kOmega * eOmega +
                         tauI;

    // ESO 扰动补偿（门限不满足时不接入）
    const float tauEsoRaw = EsoComp();
    float tauEso = 0.0f;
    if (cfg_.esoCompEnable && GatePass(fb.omega, cfg_.esoOmegaGate) &&
        GatePass(ref.alpha, cfg_.esoAlphaGate)) {
        tauEso         = tauEsoRaw;
        out.esoActive  = true;
    }

    // 偏置补偿：跟随"指令 - 实测"力矩差
    const float tauNoBias = tauLqr + tauEso;
    tauBias_ = ClampAbs(tauBias_ + cfg_.tauBiasKi * (tauNoBias - tauMeasFilt_) * dt,
                        cfg_.tauBiasLimit);
    float tau = tauNoBias + tauBias_;

    // 软限幅 → 硬限幅 → 斜率限制
    if (cfg_.softLimit > 0.0f) {
        const float limited = ClampAbs(tau, cfg_.softLimit);
        out.softLimit = (limited != tau);
        tau = limited;
    }
    if (cfg_.tauMin < cfg_.tauMax) {
        const float limited = Clamp(tau, cfg_.tauMin, cfg_.tauMax);
        out.hardLimit = (limited != tau);
        tau = limited;
    }
    if (cfg_.slewEnable && cfg_.slewRate > 0.0f) {
        const float maxDelta = cfg_.slewRate * dt;
        const float limited =
            Clamp(tau, tauCmd_ - maxDelta, tauCmd_ + maxDelta);
        out.slewLimit = (limited != tau);
        tau = limited;
    }
    tauCmd_ = tau;

    out.thetaRef  = ref.theta;
    out.omegaRef  = ref.omega;
    out.alphaRef  = ref.alpha;
    out.eTheta    = eTheta;
    out.eOmega    = eOmega;
    out.tauFf     = tauFf;
    out.tauI      = tauI;
    out.tauLqr    = tauLqr;
    out.tauEsoRaw = tauEsoRaw;
    out.tauEso    = tauEso;
    out.tauBias   = tauBias_;
    out.tauCmd    = tau;
    return out;
}

} // namespace alg::lqr_eso
