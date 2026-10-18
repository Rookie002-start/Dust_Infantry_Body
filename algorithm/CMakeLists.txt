# 项目算法层装配 — 门禁符号 ALG_XXX 为项目私有（框架算法层是 DUST_ 前缀的 submodule）

if(CONFIG_ALG_LQR_ESO)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/lqr_eso/yaw_lqr_eso.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/notch/notch.cpp
	)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/lqr_eso
		${CMAKE_CURRENT_SOURCE_DIR}/notch
	)
endif()
