/**
 * @file trd_gimbal.cpp
 * @author Rookie002-start
 * @brief 云台控制线程：双轴 LQR+ESO，1 ms 周期输出 DM 控制帧
 * @version 0.2
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Gimbal"

#include "from_head.hpp"
#include "gimbal_to.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_gimbal.hpp"
#include "can.hpp"
#include "lpf.hpp"
#include "notch.hpp"
#include "vofa.hpp"
#if defined(CONFIG_WDT_SUPERVISOR)
#include "health.hpp"
#endif

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_LATENCY_TRACE)
#include <zephyr/sys/time_units.h>
#endif
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gimbal, LOG_LEVEL_INF);

namespace thread::gimbal {

using namespace instance::gimbal;

// ==================== 结构体与枚举 ====================
enum class FbSource : uint8_t { Encoder, Imu };
enum class TestWave : uint8_t { Sine, Sweep, Steps };
enum class WaveInject : uint8_t { Position, Rate };
enum class PitchWavePhase : uint8_t { Inactive, Homing, Settle, Inject, Abort };
enum class StepPhase : uint8_t {
    Inactive,
    Homing,
    Run,
    Return,
    Settle,
    Abort,
};

struct Feedback {
    float ang   = 0.0f;
    float omega = 0.0f;
    bool  valid = false;
};

struct StepState {
    bool active_prev = false;
    StepPhase phase = StepPhase::Inactive;
    uint8_t step_index = 0;
    uint32_t phase_start_ms = 0;
    float center_angle = 0.0f;
};

struct AxisState {
    bool     enabled_prev  = false;
    bool     fb_valid_prev = false;
    bool     stop_prev     = false;
    bool     lost_prev     = false;
    DmErrorStatus err_prev = DmErrorStatus::Disable;
    uint32_t disable_ticks = 0;
    alg::filter::LowPassFilter omega_lpf {};
    float wave_base_angle = 0.0f;
    float test_hold_angle = 0.0f;
    bool test_hold_valid = false;
    StepState steps {};
};

struct LqrTuning {
    float j;                 // kg·m²
    float b;                 // N·m·s/rad
    float tauCoulomb;        // N·m
    float coulombSmooth;     // rad/s
    float kTheta;            // N·m/rad
    float kOmega;            // N·m·s/rad
    float kI;                // N·m/(rad·s)
    float intLimit;          // rad·s
    float esoBandwidth;      // rad/s
    float esoCompGain;
    float esoCompLimit;      // N·m
    float esoBlendOmegaFull; // rad/s
    float esoBlendOmegaOff;  // rad/s
    float esoBlendErrorFull; // rad
    float esoBlendErrorOff;  // rad
    float esoBlendLpfHz;     // Hz
    bool  notchEnable;
    float notchCenterHz;     // Hz
    float notchQ;
    float notchDepthDb;      // dB
};

struct AutoAimState {
    bool  enabled  = false;
    bool  off_edge = false;
    float yaw_angle = 0.0f;
    float yaw_omega = 0.0f;
    float yaw_accl  = 0.0f;
    float pitch_angle = 0.0f;
    float pitch_omega = 0.0f;
    float pitch_accl  = 0.0f;
};

struct AimFeedforward {
    float omega = 0.0f;
    float alpha = 0.0f;
};

struct TargetState {
    float yaw_cmd    = 0.0f;
    float pitch_cmd  = 0.0f;
    float yaw_target = 0.0f;
    float pitch_target = 0.0f;
    float yaw_stick_omega_ff = 0.0f;
    float yaw_wave_rate = 0.0f;
    float pitch_wave_rate = 0.0f;
};

struct AxisOutput {
    float torque = 0.0f;
    alg::lqr_eso::Output lqr {};
};

struct CommandState {
    topic::from_head::Message msg {};
    bool online_prev = false;
    bool auto_aim_prev = false;
};

struct ImuStateCache {
    topic::from_head::Message msg {};
    uint32_t last_ms = 0;
    bool valid_prev[2] = { false, false };
};

struct RxState {
    volatile uint32_t yaw_ms = 0;
    volatile uint32_t pitch_ms = 0;
};

struct ChassisYawFeedforward {
    float rate = 0.0f;
    float torque = 0.0f;
    alg::filter::LowPassFilter rate_lpf {};
};

struct TestWaveState {
    bool     on = true;
    float    phase = 0.0f;
    uint32_t sweep_t0 = 0;
    uint8_t  sweep_idx = 0;
    float    freq = 0.0f;
    float    value = 0.0f;
};

struct StepLevel {
    float value;
    uint32_t hold_ms;
};

struct StepProfile {
    const StepLevel *levels;
    uint8_t count;
    bool home_to_zero;
    float home_rate;
    float home_tolerance;
    float home_omega_tolerance;
    float safe_limit;
    float guard_time;
    float breakaway_rate;
    uint32_t kick_ms;
    uint32_t settle_ms;
    uint32_t timeout_ms;
    const char *name;
};

struct PitchWaveState {
    bool active_prev = false;
    PitchWavePhase phase = PitchWavePhase::Inactive;
    uint32_t start_ms = 0;
    uint32_t stable_since_ms = 0;
};

struct LoopState {
    uint32_t vofa_div = 0;
    uint32_t iters = 0;
    uint32_t overruns = 0;
    uint32_t body_max = 0;
    uint32_t last_ms = 0;
};

#if defined(CONFIG_LATENCY_TRACE)
struct LatencyTraceState {
    bool     seen = false;
    uint8_t  last_seq = 0;
    bool     pending = false;
    uint8_t  seq = 0;
    uint32_t rx_cycles = 0;
    uint32_t yaw_us = 0;
    uint32_t pitch_us = 0;
    bool     yaw_done = false;
    bool     pitch_done = false;
};
#endif

// ==================== 周期与机械参数 ====================
static constexpr uint32_t kPeriodMs = 1;
static constexpr float    kDt      = static_cast<float>(kPeriodMs) / 1000.0f;
static constexpr float kPi = 3.14159265f;
static constexpr float k2Pi = 2.0f * kPi;
static constexpr float kRadToDeg = 180.0f / kPi;

static constexpr float kPitchMin    = -0.300f;
static constexpr float kPitchMax    =  0.300f;
static constexpr float kJointRatio  = 1.0f;
static constexpr float kMotorVmax   = 30.0f;
static constexpr float kMotorTmax   = 10.0f;
static constexpr float kTorqueLimit = 6.0f;
static_assert(kTorqueLimit <= kMotorTmax, "torque limit exceeds motor TMAX");
static constexpr float kMaxOmegaRef = kMotorVmax / kJointRatio;
static constexpr uint32_t kEnableResendTicks = 200;
static constexpr uint32_t kLostTimeoutMs = 50;
static constexpr uint32_t kImuTimeoutMs = 20;
static constexpr uint8_t  kTxPerTickMax = 8;
static constexpr uint32_t kDropLogPeriod = 1000;
static constexpr uint32_t kVofaDiv = 5;

// ==================== 方向与反馈源 ====================
static constexpr int8_t kYawAutoSign    = -1;
static constexpr int8_t kPitchAutoSign  = +1;
static constexpr int8_t kYawMotorSign   = +1;
static constexpr int8_t kPitchMotorSign = -1;
#if CONFIG_TRD_GIMBAL_FB_IMU
static constexpr int8_t kYawImuSign     = -1;
static constexpr int8_t kPitchImuSign   = +1;
#endif

static constexpr FbSource kYawPosFb    = FbSource::Imu;
static constexpr FbSource kYawRateFb   = FbSource::Imu;
static constexpr FbSource kPitchPosFb  = FbSource::Imu;
static constexpr FbSource kPitchRateFb = FbSource::Imu;

#if !CONFIG_TRD_GIMBAL_FB_IMU
static_assert(kYawPosFb == FbSource::Encoder && kYawRateFb == FbSource::Encoder &&
              kPitchPosFb == FbSource::Encoder && kPitchRateFb == FbSource::Encoder,
              "IMU feedback selected without CONFIG_TRD_GIMBAL_FB_IMU");
#endif

// ==================== LQR+ESO 参数 ====================
// 双轴控制结构相同，参数独立整定。
static constexpr LqrTuning kYawLqr {
    .j = 0.048f,
    .b = 0.040f,
    .tauCoulomb = 0.0f,
    .coulombSmooth = 0.03f,
    .kTheta = 24.0f,
    .kOmega = 1.8f,
    .kI = 0.0f,
    .intLimit = 0.4f,
    .esoBandwidth = 30.0f,
    .esoCompGain = 0.3f,
    .esoCompLimit = 1.0f,
    .esoBlendOmegaFull = 0.20f,
    .esoBlendOmegaOff = 0.80f,
    .esoBlendErrorFull = 0.01f,
    .esoBlendErrorOff = 0.05f,
    .esoBlendLpfHz = 20.0f,
    .notchEnable = true,
    .notchCenterHz = 9.0f,
    .notchQ = 4.5f,
    .notchDepthDb = 6.0f,
};

static constexpr LqrTuning kPitchLqr {
    .j = 0.029841888f,
    .b = 0.210f,
    .tauCoulomb = 0.21f,
    .coulombSmooth = 0.03f,
    .kTheta = 27.0f,
    .kOmega = 1.5f,
    .kI = 10.0f,
    .intLimit = 0.05f,
    .esoBandwidth = 20.0f,
    .esoCompGain = 1.0f,
    .esoCompLimit = 0.2f,
    .esoBlendOmegaFull = 0.20f,
    .esoBlendOmegaOff = 0.80f,
    .esoBlendErrorFull = 0.01f,
    .esoBlendErrorOff = 0.05f,
    .esoBlendLpfHz = 20.0f,
    .notchEnable = false,
    .notchCenterHz = 8.5f,
    .notchQ = 5.0f,
    .notchDepthDb = 6.0f,
};

// ==================== 反馈滤波 ====================
static constexpr float kYawOmegaLpfHz = 100.0f;
static constexpr float kPitchOmegaLpfHz = 0.0f;

// ==================== 前馈参数 ====================
// 双轴前馈诊断开关；关闭后仅保留 LQR、ESO 和反馈闭环。
static constexpr bool kYawFeedforwardEnable = true;
static constexpr bool kPitchFeedforwardEnable = true;
static constexpr float kYawChassisRateFfGain =
    (kYawPosFb == FbSource::Imu && kYawRateFb == FbSource::Encoder) ? -1.0f : 0.0f;
static constexpr float kYawChassisTorqueFfGain =
    (kYawPosFb == FbSource::Imu && kYawRateFb == FbSource::Imu) ? 1.0f : 0.0f;
static constexpr float kYawChassisRateLpfHz = 50.0f;

static constexpr float kYawSensitivity = 0.008f;
static constexpr float kYawStickRateFfGain = 0.0f;
static constexpr float kYawStickRateGain = kYawStickRateFfGain * kYawSensitivity / kDt;

static constexpr float kAimYawOmegaFfGain = 1.0f;
static constexpr float kAimYawAlphaFfGain = 0.5f;
static constexpr float kAimPitchOmegaFfGain = 1.0f;
static constexpr float kAimPitchAlphaFfGain = 0.7f;

static constexpr float kPitchAimAlphaLpfHz = 50.0f;
static constexpr bool  kPitchAimAlphaNotchEnable = true;
static constexpr float kPitchAimAlphaNotchCenterHz = 12.0f;
static constexpr float kPitchAimAlphaNotchQ = 3.0f;
static constexpr float kPitchAimAlphaNotchDepthDb = 8.0f;

static constexpr float kPitchGravGain = -0.785f;
static constexpr float kPitchGravCos =  1.277f;
static constexpr float kPitchGravSin = -0.122f;

// ==================== 测试波形参数 ====================
// #define CONFIG_WAVE_TEST 1
static constexpr TestWave kTestWave = TestWave::Steps;
static constexpr WaveInject kWaveInject = WaveInject::Position;
static constexpr bool kYawAxisTestEnable = false;
static constexpr bool kPitchAxisTestEnable = true;

static constexpr float kTestTorqueBudget = 0.5f * kTorqueLimit;
static constexpr float kYawTestAmpPos = 0.20f;
static constexpr float kYawTestAmpRate = 2.0f;
static constexpr float kPitchTestAmpPos = 0.20f;
static constexpr float kPitchTestAmpRate = 0.30f;
static constexpr float kSinFreqHz = 2.0f;
static constexpr float kSweepFreqs[] = {
    3.0f, 4.0f, 6.0f, 7.0f, 7.5f, 8.0f, 8.5f, 9.0f,
};
static constexpr uint32_t kSweepDwellMs = 5000;

static constexpr float kTestHomeRate = 0.25f;
static constexpr float kTestHomeTolerance = 0.02f;
static constexpr float kTestHomeOmegaTolerance = 0.08f;
static constexpr float kStepRateKOmega = 3.0f;
static constexpr float kStepGuardTime = 0.10f;
static constexpr uint32_t kPitchWaveSettleMs = 100u;
static constexpr uint32_t kTestHomeTimeoutMs = 3000u;
static constexpr float kStepSafeLimit = 0.20f;
static constexpr uint32_t kStepSettleMs = 400u;
static constexpr StepLevel kPositionStepLevels[] = {
    { 0.05f, 1500u},
    {-0.05f, 1500u},
    { 0.10f, 1500u},
    {-0.10f, 1500u},
};
static constexpr StepLevel kRateStepLevels[] = {
    { 0.20f, 700u},
    {-0.20f, 700u},
    { 0.30f, 500u},
    {-0.30f, 500u},
};

static constexpr bool kWaveTestActive =
#ifdef CONFIG_WAVE_TEST
    true;
#else
    false;
#endif

static constexpr bool kWaveSteps =
#ifdef CONFIG_WAVE_TEST
    (kTestWave == TestWave::Steps);
#else
    false;
#endif

static constexpr bool kWaveSineSweep =
#ifdef CONFIG_WAVE_TEST
    (kTestWave == TestWave::Sine || kTestWave == TestWave::Sweep);
#else
    false;
#endif

static constexpr bool kYawWaveformActive =
    kWaveTestActive && kYawAxisTestEnable && kWaveSineSweep;
static constexpr bool kPitchWaveformActive =
    kWaveTestActive && kPitchAxisTestEnable && kWaveSineSweep;
static constexpr bool kStepRateInject = (kWaveInject == WaveInject::Rate);
static constexpr bool kYawStepsActive =
    kWaveTestActive && kYawAxisTestEnable && kWaveSteps;
static constexpr bool kPitchStepsActive =
    kWaveTestActive && kPitchAxisTestEnable && kWaveSteps;
static constexpr bool kVofaYawTest =
    kWaveTestActive && kYawAxisTestEnable && !kPitchAxisTestEnable;

static constexpr StepProfile kYawStepProfile {
    .levels = kStepRateInject ? kRateStepLevels : kPositionStepLevels,
    .count = static_cast<uint8_t>(
        (kStepRateInject ? sizeof(kRateStepLevels) : sizeof(kPositionStepLevels)) /
        sizeof(StepLevel)),
    .home_to_zero = false,
    .home_rate = kTestHomeRate,
    .home_tolerance = kTestHomeTolerance,
    .home_omega_tolerance = kTestHomeOmegaTolerance,
    .safe_limit = 0.0f,
    .guard_time = kStepRateInject ? kStepGuardTime : 0.0f,
    .breakaway_rate = 0.25f,
    .kick_ms = 120u,
    .settle_ms = kStepSettleMs,
    .timeout_ms = kTestHomeTimeoutMs,
    .name = "yaw",
};

static constexpr StepProfile kPitchStepProfile {
    .levels = kStepRateInject ? kRateStepLevels : kPositionStepLevels,
    .count = static_cast<uint8_t>(
        (kStepRateInject ? sizeof(kRateStepLevels) : sizeof(kPositionStepLevels)) /
        sizeof(StepLevel)),
    .home_to_zero = true,
    .home_rate = kTestHomeRate,
    .home_tolerance = kTestHomeTolerance,
    .home_omega_tolerance = kTestHomeOmegaTolerance,
    .safe_limit = kStepSafeLimit,
    .guard_time = kStepRateInject ? kStepGuardTime : 0.0f,
    .breakaway_rate = 0.25f,
    .kick_ms = 120u,
    .settle_ms = kStepSettleMs,
    .timeout_ms = kTestHomeTimeoutMs,
    .name = "pitch",
};

static_assert(kStepSafeLimit < kPitchMax &&
              -kStepSafeLimit > kPitchMin,
              "pitch step guard must stay inside soft limits");

// ==================== 运行状态 ====================
static Thread<2048> thread_ {};
static Can can3 {};
static AxisState g_yaw_state {};
static AxisState g_pitch_state {};
static TargetState g_target {};
static AutoAimState g_auto_aim {};
static AimFeedforward g_yaw_aim_ff {};
static AimFeedforward g_pitch_aim_ff {};
static AxisOutput g_yaw_output {};
static AxisOutput g_pitch_output {};
static bool gimbal_stop = false;

static CommandState g_command {};
static ImuStateCache g_imu_cache {};
static RxState g_rx {};
static LoopState g_loop {};
static uint32_t g_gimbal_version = 0;
static uint32_t g_drop = 0;
static uint32_t g_tx_fail = 0;

#if defined(CONFIG_LATENCY_TRACE)
static LatencyTraceState g_trace {};
#endif

static ChassisYawFeedforward g_chassis_yaw {};
static alg::filter::LowPassFilter g_pitch_aim_alpha_lpf {};
static alg::filter::NotchFilter g_pitch_aim_alpha_notch {};
static TestWaveState g_test_wave {};
static PitchWaveState g_pitch_wave {};

// ==================== 工具函数 ====================
static float NormalizeAngle(float angle)
{
    angle = fmodf(angle, k2Pi);
    if (angle > kPi) {
        angle -= k2Pi;
    }
    if (angle < -kPi) {
        angle += k2Pi;
    }
    return angle;
}

static float RdFloat(const uint8_t *data)
{
    float value {};
    memcpy(&value, data, sizeof value);
    return value;
}

static const char *FbName(FbSource source)
{
    return source == FbSource::Imu ? "IMU" : "ENC";
}

static float TestHoldAngle(AxisState &st, const Feedback &fb,
                           bool ready, bool reset)
{
    if (!ready) {
        st.test_hold_valid = false;
        return fb.ang;
    }
    if (reset || !st.test_hold_valid) {
        st.test_hold_angle = fb.ang;
        st.test_hold_valid = true;
    }
    return st.test_hold_angle;
}

/// pitch 重力前馈（关节系 N·m，抬头为正）
static float PitchGravityFeedforward(float angle)
{
    return kPitchGravGain *
           (kPitchGravCos * cosf(angle) + kPitchGravSin * sinf(angle));
}

static void ResetSteps(StepState &state)
{
    state = {};
}

static float RateCenterCommand(const Feedback &fb, float center_angle,
                               const StepProfile &profile)
{
    const float error = fb.ang - center_angle;
    if (error > profile.home_tolerance) {
        return -profile.home_rate;
    }
    if (error < -profile.home_tolerance) {
        return profile.home_rate;
    }
    return fb.omega > 0.0f ? -profile.home_rate : profile.home_rate;
}

/// 通用台阶状态机：按注入方式生成位置或速率命令。
static float GenerateSteps(AxisState &st, const Feedback &fb,
                           const StepProfile &profile, bool rate_injection,
                           bool active, bool reset)
{
    StepState &state = st.steps;
    const float idle_command = rate_injection ? 0.0f : fb.ang;
    if (!active) {
        if (state.active_prev) {
            ResetSteps(state);
        }
        return idle_command;
    }

    const uint32_t now = k_uptime_get_32();
    if (reset || !state.active_prev) {
        ResetSteps(state);
        state.active_prev = true;
        state.phase = StepPhase::Homing;
        state.phase_start_ms = now;
        state.center_angle = profile.home_to_zero ? 0.0f : fb.ang;
        LOG_INF("%s steps: %s injection, center=%d mrad",
                profile.name, rate_injection ? "rate" : "position",
                static_cast<int>(state.center_angle * 1000.0f));
    }

    if (state.phase == StepPhase::Abort) {
        return idle_command;
    }

    const float position = fb.ang - state.center_angle;
    if (profile.safe_limit > 0.0f &&
        state.phase != StepPhase::Homing &&
        fabsf(position) > profile.safe_limit) {
        if (rate_injection) {
            if (state.phase != StepPhase::Return) {
                state.phase = StepPhase::Return;
                state.phase_start_ms = now;
            }
        } else {
            state.phase = StepPhase::Abort;
            LOG_ERR("%s steps: safe limit exceeded, torque disabled",
                    profile.name);
        }
    }

    const bool at_center =
        fabsf(position) <= profile.home_tolerance &&
        fabsf(fb.omega) <= profile.home_omega_tolerance;

    if (state.phase == StepPhase::Homing) {
        if (at_center) {
            state.phase = StepPhase::Run;
            state.step_index = 0;
            state.phase_start_ms = now;
            LOG_INF("%s steps: center reached", profile.name);
        } else if (now - state.phase_start_ms >= profile.timeout_ms) {
            state.phase = StepPhase::Abort;
            LOG_ERR("%s steps: homing timeout, torque disabled",
                    profile.name);
        } else {
            return rate_injection
                       ? RateCenterCommand(fb, state.center_angle, profile)
                       : state.center_angle;
        }
    }

    if (state.phase == StepPhase::Return) {
        if (at_center) {
            state.phase = StepPhase::Settle;
            state.phase_start_ms = now;
            return rate_injection ? 0.0f : state.center_angle;
        }
        if (now - state.phase_start_ms >= profile.timeout_ms) {
            state.phase = StepPhase::Abort;
            LOG_ERR("%s steps: return timeout, torque disabled",
                    profile.name);
            return idle_command;
        }
        return rate_injection
                   ? RateCenterCommand(fb, state.center_angle, profile)
                   : state.center_angle;
    }

    if (state.phase == StepPhase::Settle) {
        if (now - state.phase_start_ms >= profile.settle_ms) {
            state.step_index =
                static_cast<uint8_t>((state.step_index + 1u) % profile.count);
            state.phase = StepPhase::Run;
            state.phase_start_ms = now;
            const float value = profile.levels[state.step_index].value;
            if (rate_injection) {
                LOG_INF("%s step %u: %d mrad/s", profile.name,
                        static_cast<unsigned>(state.step_index + 1u),
                        static_cast<int>(value * 1000.0f));
            } else {
                LOG_INF("%s step %u: %d mrad", profile.name,
                        static_cast<unsigned>(state.step_index + 1u),
                        static_cast<int>(value * 1000.0f));
            }
        }
        return rate_injection ? 0.0f : state.center_angle;
    }

    if (state.phase == StepPhase::Abort) {
        return idle_command;
    }

    if (now - state.phase_start_ms >=
        profile.levels[state.step_index].hold_ms) {
        state.phase = StepPhase::Return;
        state.phase_start_ms = now;
        return rate_injection
                   ? RateCenterCommand(fb, state.center_angle, profile)
                   : state.center_angle;
    }

    float command = profile.levels[state.step_index].value;
    if (rate_injection) {
        const uint32_t elapsed = now - state.phase_start_ms;
        if (command != 0.0f && elapsed < profile.kick_ms &&
            fabsf(command) < profile.breakaway_rate) {
            command = copysignf(profile.breakaway_rate, command);
        }
        if (profile.safe_limit > 0.0f) {
            const float projected = position + command * profile.guard_time;
            if (fabsf(projected) > profile.safe_limit) {
                state.phase = StepPhase::Return;
                state.phase_start_ms = now;
                command = RateCenterCommand(fb, state.center_angle, profile);
            }
        }
    } else {
        command += state.center_angle;
        if (profile.safe_limit > 0.0f) {
            command = std::clamp(command,
                                 state.center_angle - profile.safe_limit,
                                 state.center_angle + profile.safe_limit);
        }
    }

    return command;
}

static void ResetPitchWave()
{
    g_pitch_wave = {};
    g_test_wave.phase = 0.0f;
    g_test_wave.value = 0.0f;
    g_test_wave.sweep_t0 = 0;
    g_test_wave.sweep_idx = 0;
}

/// pitch 波形状态机：位置闭环回中，稳定后从 0 开始注入。
static PitchWavePhase UpdatePitchWave(const Feedback &fb, bool active, bool reset)
{
    auto &state = g_pitch_wave;
    if (!active) {
        if (state.active_prev) {
            ResetPitchWave();
        }
        return PitchWavePhase::Inactive;
    }

    const uint32_t now = k_uptime_get_32();
    if (reset || !state.active_prev) {
        ResetPitchWave();
        state.active_prev = true;
        state.phase = PitchWavePhase::Homing;
        state.start_ms = now;
        LOG_INF("pitch wave test: homing before injection");
    }

    if (state.phase == PitchWavePhase::Inject ||
        state.phase == PitchWavePhase::Abort) {
        return state.phase;
    }

    if (now - state.start_ms >= kTestHomeTimeoutMs) {
        state.phase = PitchWavePhase::Abort;
        LOG_ERR("pitch wave test: homing timeout, torque disabled");
        return state.phase;
    }

    const bool centered =
        fabsf(fb.ang) <= kTestHomeTolerance &&
        fabsf(fb.omega) <= kTestHomeOmegaTolerance;

    if (state.phase == PitchWavePhase::Homing) {
        g_test_wave.phase = 0.0f;
        g_test_wave.value = 0.0f;
        if (centered) {
            state.phase = PitchWavePhase::Settle;
            state.stable_since_ms = now;
        }
        return state.phase;
    }

    if (!centered) {
        state.phase = PitchWavePhase::Homing;
        state.stable_since_ms = 0;
        return state.phase;
    }

    if (now - state.stable_since_ms >= kPitchWaveSettleMs) {
        state.phase = PitchWavePhase::Inject;
        g_test_wave.phase = 0.0f;
        g_test_wave.value = 0.0f;
        LOG_INF("pitch wave test: centered, injection enabled");
    }
    return state.phase;
}

// ==================== 测试波形 ====================
/// 按惯量和频率限制波形幅值，避免力矩饱和。
static float TestAmpAt(float f_hz, bool pitch_axis)
{
    const bool  rate = (kWaveInject == WaveInject::Rate);
    const float j = pitch_axis ? kPitchLqr.j : kYawLqr.j;
    const float base = rate ? (pitch_axis ? kPitchTestAmpRate : kYawTestAmpRate)
                            : (pitch_axis ? kPitchTestAmpPos : kYawTestAmpPos);
    if (f_hz <= 0.01f) {
        return base;
    }
    const float w      = k2Pi * f_hz;
    const float accMax = kTestTorqueBudget / j;
    const float cap    = accMax / (rate ? w : w * w);
    return (cap < base) ? cap : base;
}

/// 生成本拍归一化测试波形；每周期只能调用一次。
[[maybe_unused]] static float WaveShape()
{
    if (!g_test_wave.on) {
        g_test_wave.value = 0.0f;
        return 0.0f;
    }

    if (kTestWave == TestWave::Sweep) {
        constexpr uint8_t n = sizeof(kSweepFreqs) / sizeof(kSweepFreqs[0]);
        if (g_test_wave.sweep_t0 == 0u) {
            g_test_wave.sweep_t0 = k_uptime_get_32();
        }
        const uint32_t now = k_uptime_get_32();
        if (now - g_test_wave.sweep_t0 >= kSweepDwellMs) {
            g_test_wave.sweep_t0 += kSweepDwellMs;
            g_test_wave.sweep_idx =
                static_cast<uint8_t>((g_test_wave.sweep_idx + 1u) % n);
            LOG_INF("sweep -> %.2f Hz (yaw amp %.4f, pitch amp %.4f)",
                    (double)kSweepFreqs[g_test_wave.sweep_idx],
                    (double)TestAmpAt(kSweepFreqs[g_test_wave.sweep_idx], false),
                    (double)TestAmpAt(kSweepFreqs[g_test_wave.sweep_idx], true));
        }
        g_test_wave.freq = kSweepFreqs[g_test_wave.sweep_idx];
    } else {
        g_test_wave.freq = kSinFreqHz;
    }

    g_test_wave.phase += k2Pi * g_test_wave.freq * kDt;
    if (g_test_wave.phase >= k2Pi) {
        g_test_wave.phase -= k2Pi;
    }

    g_test_wave.value = sinf(g_test_wave.phase);
    return g_test_wave.value;
}

/// 读取上板指令；链路失效时停机并清除速率前馈。
static void ReadCommand()
{
    auto &msg = g_command.msg;
    g_auto_aim.off_edge = false;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) != 0) {
        return;
    }

    const bool head_alive = msg.online;
    const bool remote_ok = inter_cmd::CommLinkOk(msg.comm);
    const bool cmd_ok     = head_alive && remote_ok;

    if (!cmd_ok) {
        if (g_command.online_prev) {
            LOG_WRN("gimbal command lost (head=%u link=%u flags=0x%02x) -> stop, torque 0",
                    head_alive ? 1u : 0u, remote_ok ? 1u : 0u,
                    static_cast<unsigned>(msg.comm.flags));
        }
        gimbal_stop = true;
        g_target.yaw_stick_omega_ff = 0.0f;
        g_command.online_prev = false;
        return;
    }

    if (!g_command.online_prev) {
        LOG_INF("gimbal command online");
        g_command.online_prev = true;
    }

    const bool stop_now =
        (static_cast<inter_cmd::SpinMode>(msg.comm.chassis_spin) == inter_cmd::SpinMode::Stop);
    if (stop_now != gimbal_stop) {
        LOG_INF("gimbal stop %s (torque 0, still enabled)", stop_now ? "ON" : "OFF");
    }
    gimbal_stop = stop_now;

    g_auto_aim.enabled =
        (msg.comm.Switch.AutoAim != 0u) && inter_cmd::CommVisionOk(msg.comm);
    g_auto_aim.yaw_angle = kYawAutoSign * RdFloat(msg.autoaim.yaw_angle);
    g_auto_aim.yaw_omega = kYawAutoSign * RdFloat(msg.autoaim.yaw_omega);
    g_auto_aim.yaw_accl = kYawAutoSign * RdFloat(msg.autoaim.yaw_accl);
    g_auto_aim.pitch_angle = kPitchAutoSign * RdFloat(msg.autoaim.pitch_angle);
    g_auto_aim.pitch_omega = kPitchAutoSign * RdFloat(msg.autoaim.pitch_omega);
    g_auto_aim.pitch_accl = kPitchAutoSign * RdFloat(msg.autoaim.pitch_accl);

    const bool auto_aim_now = g_auto_aim.enabled;
    g_auto_aim.off_edge = g_command.auto_aim_prev && !auto_aim_now;
    g_command.auto_aim_prev = auto_aim_now;

#if defined(CONFIG_LATENCY_TRACE)
    const uint8_t trace_seq = inter_cmd::CommTraceSeq(msg.comm);
    if (auto_aim_now && msg.trace_rx_cycles != 0u &&
        (!g_trace.seen || trace_seq != g_trace.last_seq)) {
        g_trace.seen = true;
        g_trace.last_seq = trace_seq;
        g_trace.seq = trace_seq;
        g_trace.rx_cycles = msg.trace_rx_cycles;
        g_trace.pending = true;
        g_trace.yaw_done = false;
        g_trace.pitch_done = false;
    } else if (!auto_aim_now) {
        g_trace.pending = false;
    }
#endif

    if (auto_aim_now) {
        g_target.yaw_stick_omega_ff = 0.0f;
    } else {
        g_target.yaw_cmd += msg.comm.yaw_angle * kYawSensitivity;
        g_target.yaw_stick_omega_ff = msg.comm.yaw_angle * kYawStickRateGain;
        g_target.pitch_cmd = msg.comm.pitch_angle;
        g_target.pitch_cmd *= g_target.pitch_cmd < 0.0f ? -kPitchMin : kPitchMax;
        g_target.pitch_cmd = -g_target.pitch_cmd;
    }
}

/// 由指令生成位置目标；yaw 为累计角，pitch 为绝对角。
static void UpdateTarget()
{
#ifndef CONFIG_WAVE_TEST
    if (g_auto_aim.enabled) {
        g_target.yaw_target = g_auto_aim.yaw_angle;
        g_target.pitch_target =
            std::clamp(g_auto_aim.pitch_angle, kPitchMin, kPitchMax);
    } else {
        g_target.yaw_target = g_target.yaw_cmd;
        g_target.pitch_target =
            std::clamp(g_target.pitch_cmd, kPitchMin, kPitchMax);
    }
#else
    g_target.yaw_wave_rate = 0.0f;
    g_target.pitch_wave_rate = 0.0f;
    g_target.yaw_target = 0.0f;
    g_target.pitch_target = 0.0f;

    if (kYawWaveformActive || kPitchWaveformActive) {
        const float wave = WaveShape();
        const float yaw_amp = TestAmpAt(g_test_wave.freq, false);
        const float pitch_amp = TestAmpAt(g_test_wave.freq, true);
        if (kYawWaveformActive && kWaveInject == WaveInject::Rate) {
            g_target.yaw_wave_rate = yaw_amp * wave;
        } else if (kYawWaveformActive) {
            g_target.yaw_target = yaw_amp * wave;
        }
        if (kPitchWaveformActive && kWaveInject == WaveInject::Rate) {
            g_target.pitch_wave_rate = pitch_amp * wave;
        } else if (kPitchWaveformActive) {
            g_target.pitch_target = pitch_amp * wave;
        }
    }
#endif
}

// ==================== 角度反馈 ====================

#if CONFIG_TRD_GIMBAL_FB_IMU
/// 读取上板 IMU；帧超时或上板 IMU 失效时返回无效反馈。
static Feedback ReadImuFeedback(bool yaw_axis)
{
    auto &msg = g_imu_cache.msg;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) == 0) {
        const bool ok = msg.online && inter_cmd::CommImuOk(msg.comm) &&
                        (msg.age_ms <= kImuTimeoutMs);
        g_imu_cache.last_ms = ok ? k_uptime_get_32() : 0u;
    }

    Feedback fb {};
    if (yaw_axis) {
        // 累计偏航可跨圈，不绕回；角速度取同一帧的陀螺 z 轴（上板已填到 yaw_omega）
        fb.ang   = kYawImuSign * RdFloat(msg.imu.total_yaw_angle);
        fb.omega = kYawImuSign * RdFloat(msg.imu.yaw_omega);
    } else {
        fb.ang   = kPitchImuSign * RdFloat(msg.imu.pitch_angle);
        fb.omega = kPitchImuSign * RdFloat(msg.imu.pitch_omega);
    }
    fb.valid = (g_imu_cache.last_ms != 0u) &&
               ((k_uptime_get_32() - g_imu_cache.last_ms) <= kImuTimeoutMs);

    const uint8_t idx = yaw_axis ? 0u : 1u;
    if (fb.valid != g_imu_cache.valid_prev[idx]) {
        g_imu_cache.valid_prev[idx] = fb.valid;
        LOG_INF("gimbal %s IMU feedback (upper board) %s",
                yaw_axis ? "yaw" : "pitch", fb.valid ? "online" : "STALE");
    }
    return fb;
}
#endif

/// 未启用 IMU 配置时返回空反馈。
static Feedback ReadImuFeedbackOrNone(bool yaw_axis)
{
#if CONFIG_TRD_GIMBAL_FB_IMU
    return ReadImuFeedback(yaw_axis);
#else
    (void)yaw_axis;
    return Feedback {};
#endif
}

/// 按配置选择 IMU 或编码器反馈，并统一到物理坐标系。
static Feedback ReadFeedback(const DmMotor::Snapshot &snap, const Feedback &imu_fb, bool yaw_axis)
{
    const FbSource pos_src  = yaw_axis ? kYawPosFb   : kPitchPosFb;
    const FbSource rate_src = yaw_axis ? kYawRateFb  : kPitchRateFb;
    const int8_t   sign     = yaw_axis ? kYawMotorSign : kPitchMotorSign;

    Feedback fb {};
    fb.ang   = (pos_src == FbSource::Imu)
                   ? imu_fb.ang
                   : static_cast<float>(sign) * snap.radian / kJointRatio;
    fb.omega = (rate_src == FbSource::Imu)
                   ? imu_fb.omega
                   : static_cast<float>(sign) * snap.omega / kJointRatio;
    fb.valid = (pos_src == FbSource::Imu || rate_src == FbSource::Imu) ? imu_fb.valid : true;
    return fb;
}

/// 估计底盘 yaw 角速度：世界系陀螺角速度减关节角速度。
static void UpdateChassisYawRate(const Feedback &yaw_imu_fb,
                                 const DmMotor::Snapshot &yaw_snap,
                                 bool yaw_lost)
{
    const bool ok = yaw_imu_fb.valid && !yaw_lost &&
                    (yaw_snap.err == DmErrorStatus::Enable);
    if (!ok) {
        g_chassis_yaw.rate = 0.0f;
        g_chassis_yaw.rate_lpf.Reset(0.0f);
        return;
    }

    const float omega_joint =
        static_cast<float>(kYawMotorSign) * yaw_snap.omega / kJointRatio;
    g_chassis_yaw.rate =
        g_chassis_yaw.rate_lpf.Update(yaw_imu_fb.omega - omega_joint);
}

// ==================== CAN 发送 ====================

static void QueueFrame(Axis &axis, bool cmd)
{
    topic::to_can_tx::Message msg {};
    msg.tx_id = axis.motor.GetTxId();

    if (cmd) {
        axis.motor.PackCmdFrame(msg.data, DmMotor::Cmd::Enable);
    } else {
        axis.motor.PackCtrlFrame(msg.data);
    }

    if (k_msgq_put(gimbal_tx, &msg, K_NO_WAIT) != 0) {
        if ((++g_drop % kDropLogPeriod) == 1u) {
            LOG_WRN("gimbal tx queue full: %u frames dropped",
                    static_cast<unsigned>(g_drop));
        }
    }
}

/// 发送一帧使能命令。
static void RequestEnable(Axis &axis)
{
    QueueFrame(axis, true);
}

#if defined(CONFIG_LATENCY_TRACE)
/// 在真正调用云台电机 CAN 发送前记录本轴本地耗时分母。
static void TraceBeforeMotorSend(uint16_t tx_id)
{
    if (!g_trace.pending) {
        return;
    }

    const uint32_t elapsed =
        k_cyc_to_us_floor32(k_cycle_get_32() - g_trace.rx_cycles);
    if (tx_id == kYawCanId && !g_trace.yaw_done) {
        g_trace.yaw_us = elapsed;
        g_trace.yaw_done = true;
    } else if (tx_id == kPitchCanId && !g_trace.pitch_done) {
        g_trace.pitch_us = elapsed;
        g_trace.pitch_done = true;
    }

    if (g_trace.yaw_done && g_trace.pitch_done) {
        inter_cmd::TraceState trace{};
        trace.seq        = g_trace.seq;
        trace.valid_mask = static_cast<uint8_t>(inter_cmd::kTraceValidYaw |
                                                inter_cmd::kTraceValidPitch);
        trace.yaw_us     = g_trace.yaw_us;
        trace.pitch_us   = g_trace.pitch_us;
        inter_cmd::PostFrame(inter_cmd::FrameType::StateTrace, trace);
        g_trace.pending = false;
    }
}
#endif

/// 外发 user_can3 队列。
static void FlushTx()
{
    topic::to_can_tx::Message msg {};
    uint8_t sent = 0;

    while (sent < kTxPerTickMax && k_msgq_get(gimbal_tx, &msg, K_NO_WAIT) == 0)
    {
        // 契约固定 8 字节载荷（不能用 sizeof(tx.data)：CAN FD 模式下是 64 字节）
        can_frame tx {};
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, sizeof(msg.data));

#if defined(CONFIG_LATENCY_TRACE)
        TraceBeforeMotorSend(msg.tx_id);
#endif
        if (!can3.Send(&tx)) {
            if ((++g_tx_fail % kDropLogPeriod) == 1u) {
                LOG_WRN("can3 send fail id=0x%02x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(g_tx_fail));
            }
        }
        ++sent;
    }
}

// ==================== 电机状态检测 ====================

/// 超过 kLostTimeoutMs 未收到反馈即判定掉线。
static bool RxLost(volatile uint32_t &rx_ms)
{
    return (k_uptime_get_32() - rx_ms) > kLostTimeoutMs;
}

/// 故障码转日志文本。
static const char *ErrName(DmErrorStatus e)
{
    switch (e) {
    case DmErrorStatus::Disable:        return "disabled";
    case DmErrorStatus::Enable:         return "enabled";
    case DmErrorStatus::EncoderUncalib: return "encoder-uncalibrated";
    case DmErrorStatus::OverVoltage:    return "over-voltage";
    case DmErrorStatus::UnderVoltage:   return "under-voltage";
    case DmErrorStatus::OverCurrent:    return "over-current";
    case DmErrorStatus::MosOvertemp:    return "mos-overtemp";
    case DmErrorStatus::CoilOvertemp:   return "coil-overtemp";
    case DmErrorStatus::CommLost:       return "comm-lost";
    case DmErrorStatus::Overload:       return "overload";
    }
    return "unknown";
}

// ==================== 自瞄参考前馈 ====================
static AimFeedforward YawAutoAimFeedforward()
{
    if (!g_auto_aim.enabled) {
        return {};
    }

    return {
        .omega = kAimYawOmegaFfGain * g_auto_aim.yaw_omega,
        .alpha = kAimYawAlphaFfGain * g_auto_aim.yaw_accl,
    };
}

static AimFeedforward PitchAutoAimFeedforward()
{
    if (!g_auto_aim.enabled) {
        g_pitch_aim_alpha_lpf.Reset(0.0f);
        g_pitch_aim_alpha_notch.Reset(0.0f);
        return {};
    }

    const float alpha =
        g_pitch_aim_alpha_lpf.Update(kAimPitchAlphaFfGain * g_auto_aim.pitch_accl);
    return {
        .omega = kAimPitchOmegaFfGain * g_auto_aim.pitch_omega,
        .alpha = g_pitch_aim_alpha_notch.Update(alpha),
    };
}

static float YawOmegaFeedforward()
{
#ifdef CONFIG_WAVE_TEST
    const float stick_ff = 0.0f;
#else
    const float stick_ff = g_target.yaw_stick_omega_ff;
#endif
    return kYawChassisRateFfGain * g_chassis_yaw.rate + stick_ff;
}

static float YawChassisTorqueFeedforward()
{
    return -kYawChassisTorqueFfGain * kYawLqr.b * g_chassis_yaw.rate;
}

/// 单轴状态管理、LQR 计算与电机输出。
static void ProcessAxis(Axis &axis, AxisState &st, int8_t sign,
                        float &target_angle, const Feedback &fb,
                        bool stop, bool lost, bool auto_aim_off,
                        const DmMotor::Snapshot &snap,
                        bool yaw_axis, AxisOutput &output,
                        float omega_ff = 0.0f,
                        float alpha_ff = 0.0f,
                        float tau_ff = 0.0f,
                        bool open_pos_loop = false)
{
    const bool enabled = (snap.err == DmErrorStatus::Enable);

    // 电机状态仅在上报沿打印。
    if (snap.err != st.err_prev) {
        st.err_prev = snap.err;
        if (enabled) {
            LOG_INF("gimbal axis enabled");
        } else if (snap.err == DmErrorStatus::Disable) {
            LOG_WRN("gimbal axis disabled -> requesting enable");
        } else {
            LOG_ERR("gimbal axis fault: %s (0x%x)", ErrName(snap.err),
                    static_cast<unsigned>(snap.err));
        }
    }

    const bool lost_release = st.lost_prev && !lost;
    if (lost != st.lost_prev) {
        st.lost_prev = lost;
        if (lost) {
            LOG_ERR("gimbal axis feedback LOST (%ums) -> hold, 0 torque",
                    static_cast<unsigned>(kLostTimeoutMs));
        } else {
            LOG_INF("gimbal axis feedback recovered");
        }
    }
    const bool stop_release = st.stop_prev && !stop;
    const bool align_edge = (enabled && !st.enabled_prev) ||
                            (enabled && fb.valid &&
                             (!st.fb_valid_prev || stop_release || lost_release || auto_aim_off));
    if (align_edge) {
        if (fb.valid) {
            target_angle = fb.ang;
            if (yaw_axis) {
                g_target.yaw_cmd = fb.ang;
            }
            axis.controller.Reset(fb.ang, fb.omega);
            st.omega_lpf.Reset(fb.omega);
            const char *why = stop_release ? "stop released"
                                           : (auto_aim_off ? "auto-aim released"
                                                           : (lost_release ? "feedback recovered"
                                                                           : "ready"));
            if (yaw_axis) {
                LOG_INF("gimbal yaw: %s, virtual angle re-anchored to %d mrad",
                        why, static_cast<int>(fb.ang * 1000.0f));
            } else {
                LOG_INF("gimbal pitch: %s (absolute command: follows stick next tick)", why);
            }
        } else {
            LOG_WRN("gimbal %s ready but feedback invalid -> hold", yaw_axis ? "yaw" : "pitch");
        }
    }
    st.enabled_prev = enabled;
    st.fb_valid_prev = fb.valid;
    st.stop_prev = stop;

    // 失能/掉线时保持 CAN 喂狗，必要时补发使能帧。
    if (!enabled || lost)
    {
        output = {};
        const bool resend = ((st.disable_ticks++ % kEnableResendTicks) == 0u);
        if ((snap.err == DmErrorStatus::Disable || lost) && resend) {
            RequestEnable(axis);
        } else if (!lost) {
            axis.motor.SetTargetTorque(0.0f);   // 0 力矩控制帧：纯喂狗，不驱动
            QueueFrame(axis, false);
        }
        return;
    }

    st.disable_ticks = 0;

    if (!fb.valid || stop)
    {
        output = {};
        axis.motor.SetTargetTorque(0.0f);
        QueueFrame(axis, false);
        return;
    }

#ifdef CONFIG_WAVE_TEST
    const bool waveform_test_active =
        yaw_axis ? kYawWaveformActive : kPitchWaveformActive;
    if (align_edge && fb.valid) {
        st.wave_base_angle = fb.ang;
    } else if (waveform_test_active && !open_pos_loop) {
        if (yaw_axis) {
            target_angle += st.wave_base_angle;
        } else {
            target_angle = std::clamp(target_angle, kPitchMin, kPitchMax);
        }
    }
#endif

    const float omega_fb = st.omega_lpf.Update(fb.omega);
    const alg::lqr_eso::Feedback lqr_fb {
        .theta = fb.ang,
        .omega = omega_fb,
        .tauMeas = static_cast<float>(sign) * snap.torque,
        .valid = true,
    };
    const alg::lqr_eso::Reference lqr_ref {
        .theta = open_pos_loop ? fb.ang : target_angle,
        .omega = std::clamp(omega_ff, -kMaxOmegaRef, kMaxOmegaRef),
        .alpha = open_pos_loop ? 0.0f : alpha_ff,
        .tauFf = tau_ff,
    };
    output.lqr = axis.controller.Calc(lqr_fb, lqr_ref, kDt);
    output.torque = static_cast<float>(sign) * output.lqr.tauCmd;

    axis.motor.SetTargetTorque(output.torque);
    QueueFrame(axis, false);
}

// ==================== 对外发布 ====================

/// 发布相对 yaw 和双轴反馈角。
static void PublishAngles(const Feedback &yaw_fb, const Feedback &pitch_fb,
                          float yaw_relative, bool online)
{
    topic::gimbal_to::Message g {};
    g.version      = ++g_gimbal_version;
    g.timestamp_ms = k_uptime_get_32();
    g.yaw_rad      = NormalizeAngle(yaw_relative);
    g.yaw_fb_rad   = yaw_fb.ang;          // 上板要的云台反馈 yaw（模式相关：IMU 绝对角 / 编码器关节角）
    g.pitch_fb_rad = pitch_fb.ang;        // 上板要的云台反馈 pitch
    g.online       = online;
    (void)zbus_chan_pub(&pub_gimbal_to, &g, K_NO_WAIT);
}

static void VofaPublish(const DmMotor::Snapshot &snap,
                        const Feedback &fb, float rate_ref, float sign,
                        const AxisOutput &output, const AxisState &state,
                        float gravity, float test_phase, uint8_t step_index)
{
    if (++g_loop.vofa_div != kVofaDiv)
    {
        return;
    }
    g_loop.vofa_div = 0;

    const float tau_measured = sign * snap.torque;
    // ch0..15: 时间, LQR目标角, 实测角, 反馈角速度, 速率给定,
    // 实测力矩, 重力前馈, 修正力矩, tauLqr, tauFf, tauEso, 输出力矩,
    // 低通角速度, 编码器角速度, 测试状态, 台阶序号
    const float frame[] = {
        static_cast<float>(k_uptime_get_32()) * 1e-3f,
        output.lqr.thetaRef * kRadToDeg,
        fb.ang * kRadToDeg,
        fb.omega * kRadToDeg,
        rate_ref * kRadToDeg,
        tau_measured,
        gravity,
        tau_measured - gravity,
        output.lqr.tauLqr,
        output.lqr.tauFf,
        output.lqr.tauEso,
        output.torque,
        state.omega_lpf.GetOutput() * kRadToDeg,
        sign * snap.omega / kJointRatio * kRadToDeg,
        test_phase,
        static_cast<float>(step_index),
    };
    static_assert(sizeof(frame) / sizeof(frame[0]) == 16,
                  "VOFA frame must contain exactly 16 channels");

    vofa::Send(frame, sizeof(frame) / sizeof(frame[0]));
}

/// 1 ms 固定周期主循环，按绝对截止时刻调度。
static void Task(void*, void*, void*)
{
    const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
    int64_t deadline = k_uptime_ticks() + period;   // 下一个周期的绝对截止 tick

    for (;;)
    {
#if defined(CONFIG_WDT_SUPERVISOR)
        wdg_health::Mark(wdg_health::Service::Gimbal);
#endif
        const int64_t body_t0 = k_uptime_ticks();

        ++g_loop.iters;
        if (k_uptime_get_32() - g_loop.last_ms >= 1000u) {
            g_loop.last_ms = k_uptime_get_32();
            g_loop.iters = 0;
            g_loop.overruns = 0;
            g_loop.body_max = 0;
        }

        ReadCommand();
        UpdateTarget();

        const auto yaw_snap   = yaw_.motor.ReadAll();
        const auto pitch_snap = pitch_.motor.ReadAll();

        const Feedback yaw_imu_fb   = ReadImuFeedbackOrNone(true);
        const Feedback pitch_imu_fb = ReadImuFeedbackOrNone(false);

        const Feedback yaw_fb   = ReadFeedback(yaw_snap,   yaw_imu_fb,   true);
        const Feedback pitch_fb = ReadFeedback(pitch_snap, pitch_imu_fb, false);

        const bool yaw_lost   = RxLost(g_rx.yaw_ms);
        const bool pitch_lost = RxLost(g_rx.pitch_ms);

        UpdateChassisYawRate(yaw_imu_fb, yaw_snap, yaw_lost);
        g_chassis_yaw.torque = YawChassisTorqueFeedforward();

        g_yaw_aim_ff = YawAutoAimFeedforward();
        g_pitch_aim_ff = PitchAutoAimFeedforward();

        const bool yaw_test_ready =
            (yaw_snap.err == DmErrorStatus::Enable) &&
            yaw_fb.valid && !yaw_lost && !gimbal_stop;
        const bool yaw_test_reset =
            !g_yaw_state.enabled_prev ||
            !g_yaw_state.fb_valid_prev ||
            g_yaw_state.stop_prev ||
            g_yaw_state.lost_prev;
        float yaw_omega_ff;
        float yaw_alpha_ff;
        float yaw_tau_ff;
        bool yaw_open_pos_loop = false;
        StepPhase yaw_step_phase = StepPhase::Inactive;
        if (kYawStepsActive) {
            const float command =
                GenerateSteps(g_yaw_state, yaw_fb, kYawStepProfile,
                              kStepRateInject, yaw_test_ready, yaw_test_reset);
            yaw_step_phase = g_yaw_state.steps.phase;
            if (kStepRateInject) {
                g_target.yaw_wave_rate = command;
                yaw_omega_ff = kYawFeedforwardEnable ? command : 0.0f;
                yaw_open_pos_loop = true;
            } else {
                g_target.yaw_target = command;
                yaw_omega_ff = 0.0f;
            }
            yaw_alpha_ff = 0.0f;
            yaw_tau_ff = 0.0f;
        } else if (kYawWaveformActive) {
            yaw_omega_ff =
                kYawFeedforwardEnable ? g_target.yaw_wave_rate : 0.0f;
            yaw_alpha_ff = 0.0f;
            yaw_tau_ff = 0.0f;
            yaw_open_pos_loop = kWaveInject == WaveInject::Rate;
        } else if (kWaveTestActive) {
            g_target.yaw_target =
                TestHoldAngle(g_yaw_state, yaw_fb, yaw_test_ready, yaw_test_reset);
            yaw_omega_ff = 0.0f;
            yaw_alpha_ff = 0.0f;
            yaw_tau_ff = 0.0f;
        } else {
            yaw_omega_ff =
                kYawFeedforwardEnable
                    ? YawOmegaFeedforward() + g_target.yaw_wave_rate +
                          g_yaw_aim_ff.omega
                    : 0.0f;
            yaw_alpha_ff =
                kYawFeedforwardEnable ? g_yaw_aim_ff.alpha : 0.0f;
            yaw_tau_ff =
                kYawFeedforwardEnable ? g_chassis_yaw.torque : 0.0f;
        }
        float pitch_omega_ff;
        float pitch_alpha_ff;
        bool pitch_open_pos_loop = false;
        PitchWavePhase pitch_wave_phase = PitchWavePhase::Inactive;
        StepPhase pitch_step_phase = StepPhase::Inactive;
        const bool pitch_test_ready =
            (pitch_snap.err == DmErrorStatus::Enable) &&
            pitch_fb.valid && !pitch_lost && !gimbal_stop;
        const bool pitch_test_reset =
            !g_pitch_state.enabled_prev ||
            !g_pitch_state.fb_valid_prev ||
            g_pitch_state.stop_prev ||
            g_pitch_state.lost_prev;
        if (kPitchStepsActive) {
            const float command =
                GenerateSteps(g_pitch_state, pitch_fb, kPitchStepProfile,
                              kStepRateInject, pitch_test_ready, pitch_test_reset);
            pitch_step_phase = g_pitch_state.steps.phase;
            if (kStepRateInject) {
                g_target.pitch_wave_rate = command;
                pitch_omega_ff =
                    kPitchFeedforwardEnable ? command : 0.0f;
                pitch_open_pos_loop = true;
            } else {
                g_target.pitch_target = command;
                pitch_omega_ff = 0.0f;
            }
            pitch_alpha_ff = 0.0f;
        } else if (kPitchWaveformActive) {
            pitch_wave_phase =
                UpdatePitchWave(pitch_fb, pitch_test_ready, pitch_test_reset);
            if (pitch_wave_phase == PitchWavePhase::Inject) {
                pitch_omega_ff =
                    kPitchFeedforwardEnable ? g_target.pitch_wave_rate : 0.0f;
                pitch_open_pos_loop = kWaveInject == WaveInject::Rate;
            } else {
                g_target.pitch_target = 0.0f;
                pitch_omega_ff = 0.0f;
                pitch_open_pos_loop = false;
            }
            pitch_alpha_ff = 0.0f;
        } else if (kWaveTestActive) {
            g_target.pitch_target =
                TestHoldAngle(g_pitch_state, pitch_fb,
                              pitch_test_ready, pitch_test_reset);
            pitch_omega_ff = 0.0f;
            pitch_alpha_ff = 0.0f;
        } else {
            pitch_omega_ff =
                kPitchFeedforwardEnable
                    ? g_pitch_aim_ff.omega + g_target.pitch_wave_rate
                    : 0.0f;
            pitch_alpha_ff =
                kPitchFeedforwardEnable ? g_pitch_aim_ff.alpha : 0.0f;
        }
        const float pitch_tau_ff =
            kPitchFeedforwardEnable ? PitchGravityFeedforward(pitch_fb.ang) : 0.0f;
        const bool yaw_test_stop =
            gimbal_stop ||
            (kYawStepsActive && yaw_step_phase == StepPhase::Abort);
        const bool pitch_test_stop =
            gimbal_stop ||
            (kPitchWaveformActive && pitch_wave_phase == PitchWavePhase::Abort) ||
            (kPitchStepsActive && pitch_step_phase == StepPhase::Abort);

        ProcessAxis(yaw_,   g_yaw_state,   kYawMotorSign,
                    g_target.yaw_target, yaw_fb, yaw_test_stop, yaw_lost,
                    g_auto_aim.off_edge, yaw_snap,
                    true, g_yaw_output,
                    yaw_omega_ff,
                    yaw_alpha_ff,
                    yaw_tau_ff,
                    yaw_open_pos_loop);
        ProcessAxis(pitch_, g_pitch_state, kPitchMotorSign,
                    g_target.pitch_target, pitch_fb, pitch_test_stop, pitch_lost,
                    g_auto_aim.off_edge, pitch_snap,
                    false, g_pitch_output,
                    pitch_omega_ff,
                    pitch_alpha_ff,
                    pitch_tau_ff,
                    pitch_open_pos_loop);

        FlushTx();

        float test_phase = 0.0f;
        uint8_t test_step_index = 0;
        if (kVofaYawTest) {
            if (kYawWaveformActive) {
                test_phase = static_cast<float>(PitchWavePhase::Inject);
            } else {
                test_phase = static_cast<float>(yaw_step_phase);
                test_step_index = g_yaw_state.steps.step_index;
            }
        } else {
            if (kPitchWaveformActive) {
                test_phase = static_cast<float>(pitch_wave_phase);
            } else {
                test_phase = static_cast<float>(pitch_step_phase);
                test_step_index = g_pitch_state.steps.step_index;
            }
        }
        if (kVofaYawTest) {
            VofaPublish(yaw_snap, yaw_fb, yaw_omega_ff,
                        static_cast<float>(kYawMotorSign),
                        g_yaw_output, g_yaw_state, 0.0f,
                        test_phase, test_step_index);
        } else {
            VofaPublish(pitch_snap, pitch_fb, pitch_omega_ff,
                        static_cast<float>(kPitchMotorSign),
                        g_pitch_output, g_pitch_state,
                        PitchGravityFeedforward(pitch_fb.ang),
                        test_phase, test_step_index);
        }

        const float yaw_relative =
            kYawMotorSign * yaw_snap.radian / kJointRatio;
        PublishAngles(yaw_fb, pitch_fb, yaw_relative,
                      (yaw_snap.err == DmErrorStatus::Enable) && !yaw_lost);

        const uint32_t body = static_cast<uint32_t>(k_uptime_ticks() - body_t0);
        if (body > g_loop.body_max) {
            g_loop.body_max = body;
        }

        // 超时不追赶，直接重排下一拍截止时刻。
        if (k_uptime_ticks() < deadline) {
            k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
        } else {
            ++g_loop.overruns;
            deadline = k_uptime_ticks();
        }
        deadline += period;
    }
}

/// 初始化 DM 电机，闭环全部在板端完成。
static void InitMotor(Axis &axis, uint16_t can_id, uint16_t master_id)
{
    DmMotor::Config cfg {};
    cfg.ctrl_met      = ControlMethon::Mit;
    cfg.can_id        = can_id;
    cfg.master_id     = master_id;
    cfg.gearbox_ratio = kJointRatio;
    cfg.wheel_r       = 1.0f;               // 云台不做线速度换算
    cfg.kp            = 0.0f;               // 电机内置位置环关闭：闭环在板端
    cfg.kd            = 0.0f;
    cfg.PMAX          = 12.5f;
    cfg.VMAX          = kMotorVmax;
    cfg.TMAX          = kMotorTmax;         // 协议标定，不随内环限幅改动

    axis.motor.Init(cfg);
}

static alg::lqr_eso::Config MakeLqrConfig(const LqrTuning &tuning)
{
    alg::lqr_eso::Config cfg {};
    cfg.j = tuning.j;
    cfg.b = tuning.b;
    cfg.tauCoulomb = tuning.tauCoulomb;
    cfg.coulombSmooth = tuning.coulombSmooth;
    cfg.kTheta = tuning.kTheta;
    cfg.kOmega = tuning.kOmega;
    cfg.kI = tuning.kI;
    cfg.intLimit = tuning.intLimit;
    cfg.esoBandwidth = tuning.esoBandwidth;
    cfg.esoCompGain = tuning.esoCompGain;
    cfg.esoCompLimit = tuning.esoCompLimit;
    cfg.esoBlendOmegaFull = tuning.esoBlendOmegaFull;
    cfg.esoBlendOmegaOff = tuning.esoBlendOmegaOff;
    cfg.esoBlendErrorFull = tuning.esoBlendErrorFull;
    cfg.esoBlendErrorOff = tuning.esoBlendErrorOff;
    cfg.esoBlendLpfHz = tuning.esoBlendLpfHz;
    cfg.notchEnable = tuning.notchEnable;
    cfg.notchCenterHz = tuning.notchCenterHz;
    cfg.notchQ = tuning.notchQ;
    cfg.notchDepthDb = tuning.notchDepthDb;
    if (kWaveSteps && kStepRateInject) {
        cfg.kOmega = kStepRateKOmega;
        cfg.esoCompEnable = false;
        cfg.esoCompGain = 0.0f;
        cfg.notchEnable = false;
    }
    cfg.tauMeasLpf = 0.1f;
    cfg.softLimit = kTorqueLimit;
    cfg.tauMin = -kTorqueLimit;
    cfg.tauMax = kTorqueLimit;
    return cfg;
}

bool thread_init()
{
    const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can3));
    if (!device_is_ready(dev)) {
        LOG_ERR("user_can3 not ready");
        return false;
    }

    const can_filter filter { .id = 0, .mask = 0, .flags = 0 };
    if (!can3.Init(dev, filter)) {
        LOG_ERR("user_can3 init fail");
        return false;
    }
    can3.SetRxCallback(user_can3_rx_callback);

    InitMotor(yaw_, kYawCanId, kYawMasterId);
    yaw_.controller.Init(MakeLqrConfig(kYawLqr));

    InitMotor(pitch_, kPitchCanId, kPitchMasterId);
    pitch_.controller.Init(MakeLqrConfig(kPitchLqr));

    if (kWaveSteps && kStepRateInject) {
        LOG_INF("rate steps: kOmega=%.1f, ESO compensation and notch disabled",
                (double)kStepRateKOmega);
    }

    g_yaw_state.omega_lpf.Init(kYawOmegaLpfHz, kDt);
    g_pitch_state.omega_lpf.Init(kPitchOmegaLpfHz, kDt);
    g_chassis_yaw.rate_lpf.Init(kYawChassisRateLpfHz, kDt);

    g_pitch_aim_alpha_lpf.Init(kPitchAimAlphaLpfHz, kDt);
    const alg::filter::NotchFilter::Config pitch_aim_alpha_notch_cfg {
        .enable   = kPitchAimAlphaNotchEnable,
        .centerHz = kPitchAimAlphaNotchCenterHz,
        .q        = kPitchAimAlphaNotchQ,
        .depthDb  = kPitchAimAlphaNotchDepthDb,
    };
    g_pitch_aim_alpha_notch.Init(pitch_aim_alpha_notch_cfg, 1.0f / kDt);

    RequestEnable(yaw_);
    RequestEnable(pitch_);

    LOG_INF("gimbal ready: yaw 0x%02x/0x%02x pitch 0x%02x/0x%02x, "
            "feedback yaw %s/%s pitch %s/%s (pos/rate), %u ms",
            static_cast<unsigned>(kYawCanId), static_cast<unsigned>(kYawMasterId),
            static_cast<unsigned>(kPitchCanId), static_cast<unsigned>(kPitchMasterId),
            FbName(kYawPosFb), FbName(kYawRateFb), FbName(kPitchPosFb), FbName(kPitchRateFb),
            static_cast<unsigned>(kPeriodMs));
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

// CAN 中断先记录帧到达时刻，再交给电机驱动解析。
CAN_RX_HANDLER(GIMBAL_RX, kYawMasterId, [](uint8_t *data) {
    g_rx.yaw_ms = k_uptime_get_32();
    yaw_.motor.CanCpltRxCallback(data);
}, yaw);
CAN_RX_HANDLER(GIMBAL_RX, kPitchMasterId, [](uint8_t *data) {
    g_rx.pitch_ms = k_uptime_get_32();
    pitch_.motor.CanCpltRxCallback(data);
}, pitch);

REGISTER_INIT  (thread_init,  MidInit,   High, "gimbal_init");
REGISTER_THREAD(thread_start, MidThread, High, "gimbal_start");

} // namespace thread::gimbal
