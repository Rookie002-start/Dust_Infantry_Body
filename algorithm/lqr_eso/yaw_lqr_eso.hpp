/**
 * @file yaw_lqr_eso.hpp
 * @author Rookie002-start
 * @brief 单轴 LQR + 三阶 ESO 力矩控制器（前馈 + LQR + 扰动补偿 + 死区/限幅/斜率限制）
 * @version 0.1
 * @date 2026-10-03
 *
 * 模型：J·θ̈ + B·θ̇ = τ + d，状态 [θ, ω]，输入 [τ, d]。
 * 参考南京理工大学《单轴云台最优控制器设计》：连续时间跟踪 + 前馈 + LQR + ESO 补偿链路。
 * 全程弧度制；模块内不做 ±π 环绕，跨边界由调用方折到最近等效角。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "notch.hpp"

#include <cstdint>

namespace alg::lqr_eso {

/// 控制器配置（模型参数 / 增益 / 限幅 / 开关，全部有默认值）
struct Config {
    float j             = 0.0f;      // 转动惯量 J (kg·m²)，需 > 0
    float b             = 0.0f;      // 粘性阻尼 B (N·m·s/rad)

    float kTheta        = 0.0f;      // LQR 角度增益
    float kOmega        = 0.0f;      // LQR 角速度增益
    float kI            = 0.0f;      // 误差积分增益
    float intLimit      = 0.0f;      // 误差积分限幅 (rad·s)，0 = 不限

    float tauCoulomb    = 0.0f;      // 库仑摩擦前馈幅值 (N·m)
    float coulombSmooth = 0.0f;      // 库仑摩擦平滑速度 (rad/s)

    float esoBandwidth  = 0.0f;      // ESO 带宽 ω0 (rad/s)
    float esoCompGain   = 0.0f;      // 扰动补偿增益，0 = 不补偿
    float esoCompLimit  = 0.0f;      // 扰动补偿限幅 (N·m)，0 = 不限
    float esoOmegaGate  = 0.0f;      // 补偿使能速度门限 (rad/s)，≤0 = 不限
    float esoAlphaGate  = 0.0f;      // 补偿使能参考加速度门限 (rad/s²)，≤0 = 不限
    bool  esoEnable     = true;      // ESO 观测器开关
    bool  esoCompEnable = true;      // ESO 补偿接入开关（关掉仍观测）
    float esoBlendOmegaFull = 0.0f;  // |ω| ≤ 此值：补偿权重 1 (rad/s)
    float esoBlendOmegaOff  = 0.0f;  // |ω| ≥ 此值：补偿权重 0 (rad/s)
    float esoBlendErrorFull = 0.0f;  // |eθ| ≤ 此值：补偿权重 1 (rad)
    float esoBlendErrorOff  = 0.0f;  // |eθ| ≥ 此值：补偿权重 0 (rad)
    float esoBlendLpfHz     = 20.0f; // 补偿权重平滑截止频率 (Hz)

    float tauBiasKi     = 0.0f;      // 偏置补偿积分增益
    float tauBiasLimit  = 0.0f;      // 偏置补偿限幅 (N·m)，0 = 不限
    float tauMeasLpf    = 0.0f;      // 实测力矩低通系数 [0,1]，0 = 不滤波

    float deadband      = 0.0f;      // 角度误差死区 (rad)

    float softLimit     = 0.0f;      // 力矩软限幅 (N·m)，0 = 不限
    float tauMin        = 0.0f;      // 力矩硬下限 (N·m)
    float tauMax        = 0.0f;      // 力矩硬上限 (N·m)，min < max 才生效
    float slewRate      = 0.0f;      // 力矩斜率限制 (N·m/s)
    bool  slewEnable    = false;     // 斜率限制开关

    bool  notchEnable   = false;     // 输出力矩陷波开关
    float notchCenterHz = 9.0f;      // 陷波中心频率 (Hz)
    float notchQ        = 3.5f;      // 陷波品质因数
    float notchDepthDb  = 10.0f;     // 中心频率衰减量 (dB)
};

/// 反馈量
struct Feedback {
    float theta  = 0.0f;   // 实测角 (rad)
    float omega  = 0.0f;   // 实测角速度 (rad/s)
    float tauMeas = 0.0f;  // 实测力矩 (N·m)，供偏置补偿用
    bool  valid   = true;  // false → 复位并输出 0 力矩
};

/// 连续时间参考量（由上游给出，模块内不外推）
struct Reference {
    float theta = 0.0f;    // 参考角 (rad)
    float omega = 0.0f;    // 参考角速度 (rad/s)
    float alpha = 0.0f;    // 参考角加速度 (rad/s²)
    float tauFf = 0.0f;    // 外部已知力矩前馈 (N·m)，参与模型前馈和 ESO 输入
};

/// ESO 内部状态（可观测）
struct EsoState {
    float z1 = 0.0f;   // 角度估计
    float z2 = 0.0f;   // 角速度估计
    float z3 = 0.0f;   // 总扰动估计
    float b0 = 0.0f;   // 输入增益 1/J
    float w0 = 0.0f;   // 带宽
    float beta1 = 0.0f;
    float beta2 = 0.0f;
    float beta3 = 0.0f;
};

/// 状态空间矩阵（输入按 [τ, d] 排列）
struct StateSpace {
    float a[2][2] {};
    float b[2][2] {};
    float c[2] {};
};

/// 每周期输出
struct Output {
    float thetaRef = 0.0f;
    float omegaRef = 0.0f;
    float alphaRef = 0.0f;
    float eTheta   = 0.0f;   // 位置误差
    float eOmega   = 0.0f;   // 速度误差
    float tauFf    = 0.0f;   // 总前馈
    float tauI     = 0.0f;   // 积分项
    float tauLqr   = 0.0f;   // 前馈 + LQR
    float tauEsoRaw = 0.0f;  // ESO 原始补偿
    float tauEso   = 0.0f;   // 实际接入的 ESO 补偿
    float esoWeight = 0.0f;  // 补偿接入权重 [0,1]
    float tauBias  = 0.0f;   // 偏置补偿
    float tauCmd   = 0.0f;   // 最终力矩指令
    bool  esoActive  = false;
    bool  softLimit  = false;
    bool  hardLimit  = false;
    bool  slewLimit  = false;
};

/**
 * @brief 单轴 LQR + ESO 控制器
 *
 * @code
 *   alg::lqr_eso::LqrEso ctrl({.j = 0.02f, .b = 0.1f, .kTheta = 30.0f,
 *                              .kOmega = 2.0f, .esoBandwidth = 60.0f});
 *   auto out = ctrl.Calc(fb, ref, 0.001f);
 *   motor.SetTorque(out.tauCmd);
 * @endcode
 */
class LqrEso final
{
public:
    /// 默认构造：需再调 Init/SetConfig 才有效
    LqrEso() = default;
    /// 构造即载入配置
    explicit LqrEso(const Config& cfg) { Init(cfg); }

    /// 载入配置并复位状态
    void Init(const Config& cfg);

    /// 运行中改参（立即生效，按新 J/ω0 重算 ESO 增益）
    void SetConfig(const Config& cfg)
    {
        cfg_ = cfg;
        UpdateEsoGains();
        notchSampleHz_ = 0.0f;
    }

    /// 反馈失效 / 模式切换后清历史状态，用当前测量初始化观测器
    void Reset(float theta, float omega);

    /// 每控制周期调用一次
    Output Calc(const Feedback& fb, const Reference& ref, float dt);

    /// 取文档同款状态空间 A/B/C
    static bool GetStateSpace(const Config& cfg, StateSpace& model);

    /// 当前配置 / ESO 内部状态 / 上一拍力矩指令
    const Config&   GetConfig() const { return cfg_; }
    const EsoState& GetEso()    const { return eso_; }
    float GetTauCmd() const { return tauCmd_; }

private:
    void  UpdateEsoGains();                              // 整定 ESO 增益
    void  UpdateEso(float thetaMeas, float u, float dt); // 三阶 ESO 一步观测
    void  UpdateNotchConfig(float dt);                   // 按实际控制周期配置陷波
    float UpdateEsoWeight(float eTheta, float omega, bool gatePass, float dt);
    float EsoComp() const;                               // 扰动估计 → 补偿力矩
    float CoulombComp(float omegaRef) const;             // 库仑摩擦前馈

    Config   cfg_ {};
    EsoState eso_ {};
    alg::filter::NotchFilter torqueNotch_ {};

    float intTheta_    = 0.0f;
    float tauBias_     = 0.0f;
    float tauMeasFilt_ = 0.0f;
    float tauCmd_      = 0.0f;
    float notchSampleHz_ = 0.0f;
    float esoWeight_     = 1.0f;
    bool  esoWeightReady_ = false;
    bool  ready_       = false;
};

} // namespace alg::lqr_eso
