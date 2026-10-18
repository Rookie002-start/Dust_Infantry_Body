/**
 * @file notch.hpp
 * @brief 二阶陷波滤波器（中心频率 / Q / 有限陷波深度）
 * @version 0.1
 * @date 2026-10-18
 */

#pragma once

namespace alg::filter {

/**
 * @brief 二阶陷波滤波器
 *
 * 传递函数采用 RBJ 二阶陷波结构，并通过 depthDb 限制中心频率处的残余增益，
 * 避免无限深陷波在参数或采样周期变化时产生过大的数值敏感度。
 *
 * 使用时先用初始化时的采样频率计算系数，之后每个采样周期调用 Update()。
 */
class NotchFilter final
{
public:
    struct Config {
        bool  enable  = false;
        float centerHz = 9.0f;   // 中心频率 (Hz)
        float q        = 3.5f;   // 品质因数，越大陷波越窄
        float depthDb  = 10.0f;  // 中心频率衰减量 (dB)，正值表示衰减
    };

    NotchFilter() = default;

    /// 配置并复位滤波器。centerHz < sampleHz/2 且 Q > 0 时才会启用。
    bool Init(const Config& cfg, float sampleHz);

    /// 每采样周期调用；未启用或配置无效时直通。
    float Update(float input);

    /// 清空滤波器历史状态，input 用于当前输出对齐。
    void Reset(float input = 0.0f);

    bool  IsActive() const { return active_; }
    float GetLastOutput() const { return output_; }

private:
    void Recalculate();

    Config config_ {};
    float sampleHz_ = 0.0f;
    float b0_ = 1.0f;
    float b1_ = 0.0f;
    float b2_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;
    float z1_ = 0.0f;
    float z2_ = 0.0f;
    float output_ = 0.0f;
    bool  active_ = false;
};

} // namespace alg::filter
