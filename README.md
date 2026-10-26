# Dust_Infantry_Body

RoboMaster 步兵机器人**下板**固件，基于 Zephyr + Dust 架构。

下板负责执行：驱动底盘与云台、控制发射机构，并周期性地通过 CAN FD 向上板回传
实测状态（云台角度、发射机构状态等）。

## 硬件

- 主用板卡：STM32H723VGT6（`board_dm_mc02`）
- 另支持：HPM5361（RISC-V）、STM32F407（`board_rm_c`）

## 主要业务线程

`chassis` 底盘 · `gimbal` 云台 · `imu` 惯性解算 · `remote` 遥控输入
`mcu_inter` / `inter_bus` 上下板 CAN FD 收发 · `inter_rx` 接收解析 · `vofa` 波形输出

## 编译与烧录

```bash
source ~/zephyrproject/setup_env.sh
west build -b stm32h723vgt6 -- -DBOARD_CFG=board_dm_mc02
./flash.sh jlink        # 或 dap-cmsis
```

## 说明

与上板通过 CAN FD 通信（仲裁段 1 Mbps / 数据段 2 Mbps），协议定义在
`inter_cmd/inter_cmd.hpp`，必须与 Dust_Infantry_Head 中的同名文件保持一致。
SEGGER RTT 日志、VOFA+ 波形与在线调参说明见 `docs/`。
