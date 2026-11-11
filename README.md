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
HPM 板级与 SoC 支持。

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

## 自瞄控制延迟诊断（下板侧）

下板配合上板的 `CONFIG_LATENCY_TRACE`，测量：

```text
下板 CAN 0x100 RX 中断
    -> inter_rx 发布最新指令
    -> 云台线程读取并完成控制计算
    -> 向 yaw / pitch 电机提交 CAN 控制帧
```

正常固件不包含本功能。上下板必须同时打开 `CONFIG_LATENCY_TRACE=y`，并使用相同的
`inter_cmd/inter_cmd.hpp`。上板通过 `prj.conf` 打开，下板当前可使用
`latency_trace.conf`。

### 实现方式

- [`thread/inter_rx/trd_inter_rx.cpp`](thread/inter_rx/trd_inter_rx.cpp) 在 CAN RX
  回调入口记录 `k_cycle_get_32()`。
- 500 Hz PC 帧每 2 ms 变化一次，但 CAN 1 ms 广播周期会重复同一个诊断序号。
  `inter_rx` 为 16 个序号记录“第一次到达”的 RX 周期，避免延后读取时少算一拍。
- CAN RX 中断通过 `from_head::Message::trace_rx_cycles` 把本地 RX 时刻传给云台。
- [`thread/gimbal/trd_gimbal.cpp`](thread/gimbal/trd_gimbal.cpp) 在
  `ReadCommand()` 检测诊断序号变化；`FlushTx()` 在真正调用 `can3.Send()` 前，
  分别计算 yaw 和 pitch 的本地耗时。
- 下板通过 `inter_cmd::PostFrame()` 发送 `TraceState` 分片。该分片仅存在于诊断
  布局中，使 CAN 0x200 下行帧从 16 B 扩展为 28 B。

`TraceState` 在 [`inter_cmd/inter_cmd.hpp`](inter_cmd/inter_cmd.hpp) 中定义：

```text
seq         诊断序号低 4 位
valid_mask  yaw/pitch 有效位
yaw_us      CAN RX 中断 -> yaw 控制帧提交
pitch_us    CAN RX 中断 -> pitch 控制帧提交
```

下板本身不打印延迟统计，最终配对和 p50/p95/p99 汇总在上板完成，并通过上板
RTT/串口输出。

### 构建与烧录

```bash
source ~/zephyrproject/setup_env.sh
west build -p always -b stm32h723vgt6 -d build-latency -- \
  -DBOARD_CFG=board_dm_mc02 \
  -DEXTRA_CONF_FILE=latency_trace.conf
grep '^CONFIG_LATENCY_TRACE=y$' build-latency/zephyr/.config
west flash -d build-latency
```

这个配置必须与上板诊断固件成对烧录。若只更新上板或只更新下板，可能出现没有
`LT total`、配对样本很少或协议布局不一致。

### 使用条件

- 上板 PC 数据约为 500 Hz，自瞄模式保持 `mode=1` 或 `mode=2`。
- 遥控自瞄开关打开，`CommLinkOk()` 和 `CommVisionOk()` 有效。
- yaw/pitch 电机已使能，反馈正常，云台没有停机和故障。
- 诊断期间不要在云台或 CAN RX 高频路径里增加同步日志。
- 先丢弃启动和模式切换阶段，再采集至少 30 秒。

### 日志与实测结果

在上板 RTT 中查看：

```text
LT total ... yaw n=... avg(u/l/t)=... p50=... p95=... p99=... max=... us
LT total ... pitch n=... avg(u/l/t)=... p50=... p95=... p99=... max=... us
```

当前板卡稳定运行时，下板段约为：

```text
yaw 平均：1.28~1.50 ms
pitch 平均：1.28~1.51 ms
下板 yaw/pitch 差：通常约 5~10 us
```

两者差异来自 yaw/pitch 两个电机 CAN 控制帧依次发送。每 2 秒 `n` 低于 1000
通常与 PC 发送频率抖动、上板 1 ms 聚合和 level 通道覆盖有关；应结合上板实际
PC 接收计数分析，不能直接等同于下板丢帧。

完整的上板分段、总延迟、p95/p99、协议布局和看门狗注意事项见
[上板延迟诊断说明](../Dust_Infantry_Head/docs/latency_trace.md) 和
[上板 README](../Dust_Infantry_Head/README.md)。

### 与看门狗同时运行

延迟诊断增加了 500 Hz CAN 下行回传和云台统计提交。若同时启用
`CONFIG_WDT_SUPERVISOR`，某个高频服务在约 500 ms 内没有完成一次健康标记，监督器
就会锁存故障并停止喂狗，约 2 秒后复位。重启后若同样负载再次出现，会表现为周期
复位。

下板 fault mask：

| bit | 服务 |
|---:|---|
| `0x01` | Chassis |
| `0x02` | Gimbal |
| `0x04` | McuInter |
| `0x08` | InterRx |

应先读取 `watchdog health fault mask` 或 `watchdog pre-reset fault`。有非零 mask
时表示监督器主动停喂；mask 为 0 仍复位时再检查监督线程和硬件喂狗。延迟测量应在
关闭 `CONFIG_WDT_SUPERVISOR` 的独立诊断构建中进行，正式固件再单独验证看门狗。
