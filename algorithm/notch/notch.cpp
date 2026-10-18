/**
 * @file notch.cpp
 * @author Rookie002-start
 * @brief 二阶陷波滤波器实现
 * @version 0.1
 * @date 2026-10-18
 */

#pragma message "Compiling Algorithm/Filter/Notch"

#include "notch.hpp"

#include <math.h>

namespace alg::filter {

namespace {
constexpr float kPi = 3.14159265358979323846f;

inline float Clamp(float value, float minValue, float maxValue)
{
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}
} // namespace

bool NotchFilter::Init(const Config& cfg, float sampleHz)
{
    config_ = cfg;
    sampleHz_ = sampleHz;
    Recalculate();
    Reset(0.0f);
    return active_;
}

void NotchFilter::Recalculate()
{
    active_ = false;
    b0_ = 1.0f;
    b1_ = 0.0f;
    b2_ = 0.0f;
    a1_ = 0.0f;
    a2_ = 0.0f;

    const float nyquist = 0.5f * sampleHz_;
    if (!config_.enable || !isfinite(sampleHz_) || sampleHz_ <= 0.0f ||
        config_.centerHz <= 0.0f || config_.centerHz >= nyquist ||
        !isfinite(config_.centerHz) || config_.q <= 0.0f ||
        !isfinite(config_.q) || !isfinite(config_.depthDb)) {
        return;
    }

    const float w0 = 2.0f * kPi * config_.centerHz / sampleHz_;
    const float cosW0 = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * config_.q);
    const float a0Raw = 1.0f + alpha;

    if (a0Raw <= 0.0f || !isfinite(a0Raw)) {
        return;
    }

    // RBJ 陷波滤波器的归一化系数，Hn(f0) = 0。
    const float an1 = -2.0f * cosW0 / a0Raw;
    const float an2 = (1.0f - alpha) / a0Raw;
    const float bn0 = 1.0f / a0Raw;
    const float bn1 = -2.0f * cosW0 / a0Raw;
    const float bn2 = 1.0f / a0Raw;

    // H = D + (1-D)*Hn：中心处增益为 D，远离中心时保持单位增益。
    const float depthDb = Clamp(config_.depthDb, 0.0f, 60.0f);
    const float centerGain = powf(10.0f, -depthDb / 20.0f);
    const float notchMix = 1.0f - centerGain;

    b0_ = centerGain + notchMix * bn0;
    b1_ = centerGain * an1 + notchMix * bn1;
    b2_ = centerGain * an2 + notchMix * bn2;
    a1_ = an1;
    a2_ = an2;
    active_ = true;
}

float NotchFilter::Update(float input)
{
    if (!active_) {
        output_ = input;
        return output_;
    }

    // Direct Form II Transposed，数值状态少且适合浮点 MCU。
    const float output = b0_ * input + z1_;
    z1_ = b1_ * input - a1_ * output + z2_;
    z2_ = b2_ * input - a2_ * output;
    output_ = output;

    if (!isfinite(output_) || !isfinite(z1_) || !isfinite(z2_)) {
        Reset(input);
    }
    return output_;
}

void NotchFilter::Reset(float input)
{
    z1_ = 0.0f;
    z2_ = 0.0f;
    output_ = input;
}

} // namespace alg::filter
