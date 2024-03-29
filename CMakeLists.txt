cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(linux-example-btstack-handsfree-unit VERSION 1.0.0 DESCRIPTION "Linux CE Handsfree Unit Application")

# specify the C++ standard
set (CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

set (ARM_GCC_PATH "COMPONENT_ARM_LINUX/COMPONENT_GCC/")
set (BUILD_SHARED_LIBS ON)
set (BTSTACK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../btstack/wiced_include)
set (BTSTACK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (PORTING_LAYER ${CMAKE_CURRENT_SOURCE_DIR}/../bluetooth-linux)
set (BT_AUDIO_COMPONENT ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles)
set (HFU_AUDIO_COMPONENT ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/COMPONENT_handsfree_profile)
set (ALSA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../bluetooth-linux/linux-audio/lib/${ARM_GCC_PATH})
set (ALSA_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../bluetooth-linux/linux-audio/include)
set (SBC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/sbc/include)
set (SBC_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/sbc/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)

add_definitions(-DLINUX_PLATFORM -DWICED_BT_HFP_HF_WBS_INCLUDED=TRUE)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Doff64_t=__off64_t")

link_directories(${BTSTACK_LIB}/)
link_directories(${ALSA_LIB}/)
link_directories(${SBC_LIB}/)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/handsfree_unit_bt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/handsfree_unit_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/handsfree_unit_evt_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/handsfree_unit_api.c
	${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/app_bt_utils.c
	${CMAKE_CURRENT_SOURCE_DIR}/app/audio_platform_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/alsa_capture.c
	${PORTING_LAYER}/patch_download.c
    ${PORTING_LAYER}/wiced_bt_app.c
    ${PORTING_LAYER}/hci_uart_linux.c
    ${PORTING_LAYER}/linux_main.c
    ${PORTING_LAYER}/spy_ux.c
    ${PORTING_LAYER}/linux_tcp_server.c
    ${PORTING_LAYER}/wiced_hal_nvram.c
    ${PORTING_LAYER}/nvram.c
    ${PORTING_LAYER}/utils_arg_parser.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_act.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_api.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_at.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_cmd.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_main.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_ssm.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_utils.c
    ${BT_AUDIO_COMPONENT}/utils/wiced_bt_utils.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${PORTING_LAYER}/wiced_hal/)
include_directories(${BT_AUDIO_COMPONENT}/profiles_include)
include_directories(${BT_AUDIO_COMPONENT}/utils)
include_directories(${HFU_AUDIO_COMPONENT}/)
include_directories(${ALSA_INCLUDE}/)
include_directories(${SBC_INCLUDE}/)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)
target_link_libraries(${PROJECT_NAME} PRIVATE asound)
target_link_libraries(${PROJECT_NAME} PRIVATE sbc)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
