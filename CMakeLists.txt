cmake_minimum_required(VERSION 3.20.0)

# ---- (0) 框架与 zephyr_user 路径（位置：/home/rookie/zephyrproject/projects/temp）---------
set(FW_ROOT         "${CMAKE_CURRENT_SOURCE_DIR}/../../zephyr_user/framework")
set(ZEPHYR_USER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../zephyr_user")

# ---- (1) 业务门禁符号 -----------------------------------------------------
set(CONFIG_SYM PRJ_MAIN)

# ---- (2) BOARD_ROOT / DTS_ROOT 指向共享 zephyr_user 板卡层 ----------------
# 板卡专属路径（如 HPM 的 sdk_glue）由 boards/<board>/board.cmake 追加
list(APPEND BOARD_ROOT "${ZEPHYR_USER_DIR}")
list(APPEND DTS_ROOT   "${ZEPHYR_USER_DIR}")
list(APPEND DTS_ROOT   "${ZEPHYR_USER_DIR}/dts")

# ---- (3) ZEPHYR_EXTRA_MODULES（通用：framework + platform/cmsis）------------
set(ZEPHYR_EXTRA_MODULES
  "${FW_ROOT}"
  "${ZEPHYR_USER_DIR}/platform/cmsis"
  # 工程自带的 SEGGER RTT 模块（SEGGER 源码 + Zephyr 版配置头），
  # 构建/链接胶水直接复用 $ZEPHYR_BASE/modules/segger 里 Zephyr 自带的那份
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/segger"
  # 工程自带的 VOFA+ 波形输出组件（JustFloat，串口 = 设备树别名 vofa-uart）
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/vofa+"
  # 工程自带的调参终端模块（RTT 收命令行改双环 PID 的 kp/ki，业务侧只 include dbg.hpp）
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/dbg"
)
set(ZEPHYR_SDK_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../zephyr-sdk-1.0.1" CACHE PATH "")

# ---- (4) 板级 overlay / conf / board.cmake ---------------------------------
if(NOT DEFINED BOARD_CFG)
  set(BOARD_CFG ${BOARD})
endif()
file(GLOB OVERLAY_FILES   ${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_CFG}/${BOARD}.overlay)
if(OVERLAY_FILES)
  list(POP_FRONT OVERLAY_FILES DTC_OVERLAY_FILE)
endif()
file(GLOB PRJ_CONF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_CFG}/${BOARD}.conf)
if(PRJ_CONF_FILES)
  list(POP_FRONT PRJ_CONF_FILES EXTRA_CONF_FILE)
endif()
file(GLOB BOARD_CMAKE   ${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_CFG}/board.cmake)
if(BOARD_CMAKE)
  include(${BOARD_CMAKE})
endif()

# ---- (5) 接入 Zephyr 构建系统 ---------------------------------------------
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(temp)

# ---- (6) 业务源码与系统入口 ------------------------------------------------
# main() 由架构层 init 提供（framework/init/main.c），业务项目不再有 src/
target_include_directories(app PRIVATE
  ${FW_ROOT}/init
  ${FW_ROOT}/cmd
)

# 框架把 PID 的 include 目录只 PRIVATE 加给了 app，而 modules/dbg（公共调参模块）
# 的头文件里要用框架的 alg::pid::Pid —— 这里补一份全局的，模块库与业务都能 include。
zephyr_include_directories(${FW_ROOT}/algorithm/controller/pid)

# 板卡专属全局 include（由 boards/<board>/board.cmake 的 BOARD_GLOBAL_INCLUDES 提供）
foreach(_dir IN LISTS BOARD_GLOBAL_INCLUDES)
  zephyr_include_directories(${_dir})
endforeach()
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)

# ---- (6.0) Watchdog supervisor 与 SoC 适配 --------------------------------
if(CONFIG_WDT_SUPERVISOR)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/watchdog)
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog/supervisor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog/health.cpp
  )
endif()

# ---- (6.1) SEGGER RTT：补上 printf API 与头文件路径 -------------------------
# SEGGER_RTT.c / SEGGER_RTT_zephyr.c 由 Zephyr 自带的模块胶水编译，
# 但 SEGGER_RTT_printf() 不在其中，这里单独加进来，方便业务代码打变量：
#     SEGGER_RTT_printf(0, "chassis: %d %d\n", vx, vy);
if(CONFIG_USE_SEGGER_RTT)
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/segger/SEGGER/SEGGER_RTT_printf.c
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/segger/SEGGER
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/segger/Config
  )
endif()

# ---- (7) 装配架构层各层（门禁打开时 add_subdirectory）----------------------
if(CONFIG_${CONFIG_SYM})
  add_subdirectory(${FW_ROOT}/drivers   ${CMAKE_CURRENT_BINARY_DIR}/framework/drivers)
  add_subdirectory(${FW_ROOT}/algorithm ${CMAKE_CURRENT_BINARY_DIR}/framework/algorithm)
  add_subdirectory(${FW_ROOT}/modules   ${CMAKE_CURRENT_BINARY_DIR}/framework/modules)
  add_subdirectory(${FW_ROOT}/topic     ${CMAKE_CURRENT_BINARY_DIR}/framework/topic)
  add_subdirectory(${FW_ROOT}/cmd       ${CMAKE_CURRENT_BINARY_DIR}/framework/cmd)
  add_subdirectory(${FW_ROOT}/init      ${CMAKE_CURRENT_BINARY_DIR}/framework/init)
  add_subdirectory(thread)
  add_subdirectory(algorithm)
  add_subdirectory(my_topic)
  add_subdirectory(inter_cmd)
endif()
