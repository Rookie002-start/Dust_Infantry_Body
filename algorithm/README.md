# 项目算法层

项目私有的纯计算模块（框架算法层见 `zephyr_user/framework/algorithm`，是 `DUST_` 前缀的 submodule）。
新增算法在对应目录下建 `xxx.hpp/.cpp`，在本目录 `CMakeLists.txt` 与 `Kconfig` 注册即可。

## lqr_eso — 单轴 LQR + ESO 控制器

`alg::lqr_eso::LqrEso`：模型 `J·θ̈ + B·θ̇ = τ + d`，链路为
前馈(`J·α_ref + B·ω_ref + 库仑`) → LQR 反馈(`kTheta/kOmega/kI`) → 三阶 ESO 扰动补偿
→ 偏置补偿 → 软/硬限幅 → 斜率限制，输出力矩指令。
参考南理工（NJUST）《单轴云台最优控制器设计》，Kconfig 门禁 `CONFIG_ALG_LQR_ESO`。

```cpp
#include "yaw_lqr_eso.hpp"

alg::lqr_eso::LqrEso ctrl({
    .j = 0.021f, .b = 0.09f, .kTheta = 32.0f, .kOmega = 2.6f, .kI = 0.6f,
    .intLimit = 0.4f, .esoBandwidth = 60.0f, .esoCompGain = 1.0f,
    .esoCompLimit = 1.5f, .softLimit = 2.0f, .tauMin = -3.0f, .tauMax = 3.0f,
    .slewRate = 200.0f, .slewEnable = true,
});

// 反馈失效时先复位，再每周期调用
alg::lqr_eso::Feedback fb { .theta = yaw, .omega = yawRate, .tauMeas = tau, .valid = ok };
alg::lqr_eso::Reference ref { .theta = yawRef, .omega = yawRefRate, .alpha = yawRefAcc };
float tauCmd = ctrl.Calc(fb, ref, 0.001f).tauCmd;
```

要点：角度统一弧度；参考量由上游给连续 `theta/omega/alpha`，模块不外推；
跨 `±π` 由调用方折到最近等效角；首个有效周期只同步状态、输出 0 力矩。
