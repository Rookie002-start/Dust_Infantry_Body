# Dust_Infantry_Body

RoboMaster 步兵机器人**下板**固件，基于 Zephyr + Dust 架构。下板负责底盘、云台、
发射机构控制和功率估计，并通过 CAN FD 向上板回传实测状态。

## 仓库与目录

下板是与上板平级的应用仓库，不依赖上板源码。当前开发环境使用以下目录布局：

```text
~/zephyrproject/
├── zephyr/                         # 社区 Zephyr
├── modules/                        # Zephyr 社区模块
├── zephyr_user/                    # Dust_Zephyr_Tree
└── projects/
    ├── Dust_Infantry_Head/         # 上板仓库
    └── Dust_Infantry_Body/         # 本仓库

~/Zephyr_HPMicro/
├── sdk_env/                        # 社区 sdk_env，内含 hpm_sdk
├── sdk_glue/                       # 社区 zephyr_sdk_glue
└── sdk_glue_user/                  # Dust_Zephyr_HPMicro_Tree
```

`Dust_Zephyr_Tree` 提供公共框架和共享板卡定义，`Dust_Zephyr_HPMicro_Tree` 提供
HPM 板级与 SoC 支持。`custom_boards` 已不再使用，STM32H723VGT6 定义位于
`zephyr_user/boards/st/stm32h723vgt6/`。

## 硬件与板卡配置

- STM32H723VGT6：板卡 `stm32h723vgt6`，下板配置 `board_dm_mc02`。
- HPM5361：板卡 `hpm5361icb`。
- STM32F407：板卡 `stm32f407igh6`，下板配置 `board_rm_c`。

## 主要业务线程

- `chassis`：底盘控制、功率限制和 RLS 估计
- `gimbal`：云台控制、LQR/ESO 与陷波
- `imu`：惯性数据与姿态
- `remote`：遥控输入
- `mcu_inter` / `inter_bus` / `inter_rx`：上下板 CAN FD 收发与解析
- `vofa`：VOFA+ 波形输出
- `watchdog`：任务健康检查和硬件看门狗监督

## 编译

```bash
source ~/zephyrproject/setup_env.sh
west build -p always -b stm32h723vgt6 -d build -- -DBOARD_CFG=board_dm_mc02
```

该 STM32H723 配置已通过完整构建验证。HPM5361 使用：

```bash
west build -p always -b hpm5361icb -d build -- -DBOARD_CFG=hpm5361icb
```

## 烧录与调试

```bash
./flash.sh jlink        # J-Link
./flash.sh dap-cmsis    # CMSIS-DAP
```

RTT、VOFA+ 和调参终端说明见 `docs/`。RTT 控制文件位于
`/tmp/dust_infantry_body_rtt_openocd.*`。

## 上下板协议

上下板使用 CAN FD，仲裁段 1 Mbps、数据段 2 Mbps。协议当前以
`inter_cmd/inter_cmd.hpp` 的两份源码分别维护，修改时必须同步到
`Dust_Infantry_Head/inter_cmd/inter_cmd.hpp`。
