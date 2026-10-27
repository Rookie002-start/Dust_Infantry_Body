# HPM 平台：把 sdk_glue / sdk_glue_user 追加进 BOARD/SOC/DTS 根路径，并挂为 Zephyr 模块

# ---- 使用先楫专用 OpenOCD（带 hpm_xpi flash 驱动）----------------------------
# 系统自带的 openocd（apt / Zephyr SDK）没有 hpm_xpi 驱动，west flash 会直接报
#   Error: flash driver 'hpm_xpi' not found
# sdk_glue_user 的 board.cmake 是用 find_program() 从 PATH 里找 openocd 的，
# 所以这里在 find_package(Zephyr) 之前把它插到 PATH 最前面。
#   优先 ~/openocd-hpm      —— 自编译版，hpm_xpi + jlink 都有（用 J-Link 烧录用这个）
#   其次 ~/hpm-openocd      —— 先楫官方版，有 cmsis-dap/ftdi 但没有 jlink
#   可用 HPM_OPENOCD_DIR 覆盖
if(DEFINED ENV{HPM_OPENOCD_DIR})
  set(_HPM_OPENOCD_BIN "$ENV{HPM_OPENOCD_DIR}/bin")
elseif(EXISTS "$ENV{HOME}/openocd-hpm/bin/openocd")
  set(_HPM_OPENOCD_BIN "$ENV{HOME}/openocd-hpm/bin")
else()
  set(_HPM_OPENOCD_BIN "$ENV{HOME}/hpm-openocd/bin")
endif()
if(EXISTS "${_HPM_OPENOCD_BIN}/openocd")
  set(ENV{PATH} "${_HPM_OPENOCD_BIN}:$ENV{PATH}")
  message(STATUS "使用先楫 OpenOCD: ${_HPM_OPENOCD_BIN}/openocd")
else()
  message(WARNING "未找到先楫 OpenOCD (${_HPM_OPENOCD_BIN}/openocd)，烧录会报 hpm_xpi 驱动缺失")
endif()

if(DEFINED ENV{SDK_GLUE_DIR})
  set(SDK_GLUE_DIR "$ENV{SDK_GLUE_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../Zephyr_HPMicro/sdk_glue")
  set(SDK_GLUE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Zephyr_HPMicro/sdk_glue")
else()
  set(SDK_GLUE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Zephyr_HPMicro/sdk_glue")
endif()
if(DEFINED ENV{SDK_GLUE_USER_DIR})
  set(SDK_GLUE_USER_DIR "$ENV{SDK_GLUE_USER_DIR}")
else()
  get_filename_component(_HPM_ROOT "${SDK_GLUE_DIR}/.." ABSOLUTE)
  set(SDK_GLUE_USER_DIR "${_HPM_ROOT}/sdk_glue_user")
endif()
if(EXISTS "${SDK_GLUE_DIR}" AND EXISTS "${SDK_GLUE_USER_DIR}")
  list(APPEND BOARD_ROOT "${SDK_GLUE_DIR}")
  list(APPEND SOC_ROOT   "${SDK_GLUE_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_DIR}/dts")
  list(APPEND BOARD_ROOT "${SDK_GLUE_USER_DIR}")
  list(APPEND SOC_ROOT   "${SDK_GLUE_USER_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}/dts")
  list(APPEND ZEPHYR_EXTRA_MODULES "${SDK_GLUE_DIR}")
else()
  message(FATAL_ERROR
    "HPM Zephyr support not found.\n"
    "  SDK_GLUE_DIR=${SDK_GLUE_DIR}\n"
    "  SDK_GLUE_USER_DIR=${SDK_GLUE_USER_DIR}\n"
    "Run west update or set the corresponding environment variables.")
endif()

macro(app_set_runner_args)
  board_runner_args(openocd "--cmd-pre-init=adapter speed 500")
endmacro()
