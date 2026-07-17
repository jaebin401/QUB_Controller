cmake_minimum_required(VERSION 3.16)
project(QUB_Controller CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g -Wall -Wextra -fsanitize=address")

add_compile_definitions(_GNU_SOURCE)

# =============================================================================
# PCANBasic
# =============================================================================
find_path(PCAN_INCLUDE_DIR NAMES PCANBasic.h
    PATHS /usr/include /usr/local/include ${PCAN_INCLUDE_DIR})
find_library(PCAN_LIB NAMES pcanbasic
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib ${PCAN_LIB_DIR})

if(NOT PCAN_INCLUDE_DIR OR NOT PCAN_LIB)
    message(FATAL_ERROR
        "PCANBasic 헤더/라이브러리를 찾지 못했습니다.\n"
        "  cmake -DPCAN_INCLUDE_DIR=<path> -DPCAN_LIB_DIR=<path>")
endif()
message(STATUS "PCAN include: ${PCAN_INCLUDE_DIR}")
message(STATUS "PCAN lib:     ${PCAN_LIB}")

# =============================================================================
# mip_sdk (Step-4 IMU)
#
# NOTE: 2026-07 기준 HBK-MicroStrain/mip_sdk (LORD-MicroStrain은 레거시)로 이전됨.
#       헤더가 include/microstrain/cpp/mip/ 밑으로 이동했고,
#       라이브러리가 libmip.a / libmicrostrain.a / libmicrostrain_serial.a 로 분리됨.
#
# NUC에서 빌드 방법:
#   cd mip_sdk && mkdir build && cd build
#   cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mip_sdk
#   make -j$(nproc) && make install
# 그 후:
#   cmake .. -DMIP_SDK_ROOT=/opt/mip_sdk
# =============================================================================
if(DEFINED MIP_SDK_ROOT)
    find_path(MIP_INCLUDE_DIR
        NAMES mip/mip_interface.hpp
        PATHS "${MIP_SDK_ROOT}/include/microstrain/cpp"
        NO_DEFAULT_PATH
    )
    find_path(MIP_C_INCLUDE_DIR
        NAMES mip/mip_cmdqueue.h
        PATHS "${MIP_SDK_ROOT}/include/microstrain/c"
        NO_DEFAULT_PATH
    )
    find_library(MIP_LIB
        NAMES mip
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MIP_EXTRAS_LIB
        NAMES mip_extras
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MIP_METADATA_LIB
        NAMES mip_metadata
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MICROSTRAIN_LIB
        NAMES microstrain
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MICROSTRAIN_EXTRAS_LIB
        NAMES microstrain_extras
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MICROSTRAIN_SERIAL_LIB
        NAMES microstrain_serial
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MICROSTRAIN_SOCKET_LIB
        NAMES microstrain_socket
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    find_library(MICROSTRAIN_RECORDING_LIB
        NAMES microstrain_recording_connection
        PATHS "${MIP_SDK_ROOT}/lib"
        NO_DEFAULT_PATH
    )
    if(MIP_INCLUDE_DIR AND MIP_C_INCLUDE_DIR AND MIP_LIB AND MICROSTRAIN_LIB AND MICROSTRAIN_SERIAL_LIB)
        set(MIP_SDK_FOUND TRUE)
        # 순환 의존성(mip <-> microstrain <-> microstrain_serial 등) 해결을 위해
        # --start-group/--end-group으로 묶어 링커가 반복 탐색하게 함.
        # (2026-07-04 현장에서 /opt/mip_sdk/lib 확인 결과 8개 라이브러리 모두 존재 확인됨)
        set(MIP_LIBS_RAW ${MIP_LIB} ${MIP_EXTRAS_LIB} ${MIP_METADATA_LIB}
                         ${MICROSTRAIN_LIB} ${MICROSTRAIN_EXTRAS_LIB}
                         ${MICROSTRAIN_SERIAL_LIB} ${MICROSTRAIN_SOCKET_LIB}
                         ${MICROSTRAIN_RECORDING_LIB})
        set(MIP_LIBS -Wl,--start-group ${MIP_LIBS_RAW} -Wl,--end-group)
        set(MIP_INCLUDE_DIRS ${MIP_INCLUDE_DIR} ${MIP_C_INCLUDE_DIR})
        message(STATUS "mip_sdk include: ${MIP_INCLUDE_DIRS}")
        message(STATUS "mip_sdk libs:     ${MIP_LIBS_RAW}")
    else()
        message(WARNING "MIP_SDK_ROOT 지정됐지만 헤더/라이브러리를 찾지 못함. IMU 빌드 제외.")
    endif()
else()
    message(STATUS "mip_sdk: 미지정 (cmake -DMIP_SDK_ROOT=<path>). IMU 빌드 제외.")
endif()

# =============================================================================
# ONNX Runtime (Step-5 Policy)
# =============================================================================
if(DEFINED ONNXRUNTIME_ROOT)
    set(ORT_INCLUDE "${ONNXRUNTIME_ROOT}/include")
    set(ORT_LIB    "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_ROOT}")
else()
    message(STATUS "ONNX Runtime: 미지정. Policy 빌드 제외.")
endif()

# =============================================================================
# 핵심 라이브러리: qub_hw (PCAN + 모터 스레드)
# =============================================================================
add_library(qub_hw STATIC
    src/pcan_channel.cpp
    src/robstride.cpp
    src/motor_thread.cpp
)
target_include_directories(qub_hw PUBLIC include ${PCAN_INCLUDE_DIR})
target_link_libraries(qub_hw PUBLIC ${PCAN_LIB} pthread rt)

# =============================================================================
# IMU 라이브러리: qub_imu (mip_sdk 있을 때만 빌드)
# =============================================================================
if(MIP_SDK_FOUND)
    add_library(qub_imu STATIC
        src/imu_driver.cpp
    )
    target_include_directories(qub_imu PUBLIC include ${MIP_INCLUDE_DIRS})
    # mip_sdk(libmip.a 등)가 MICROSTRAIN_USE_STD_ENDIAN 정의된 상태로 빌드되어 Endian = std::endian을
    # 쓰는데, 이 매크로가 없으면 자체 enum class Endian으로 달라져 extract() 심볼이 맵글링되어
    # undefined reference가 난다 (readwrite.hpp 참고, 2026-07-04 현장에서 확인).
    # 따라서 동일하게 정의해야 함.
    target_compile_definitions(qub_imu PUBLIC MICROSTRAIN_USE_STD_ENDIAN)
    target_link_libraries(qub_imu PUBLIC ${MIP_LIBS} pthread rt)
endif()

# =============================================================================
# 실행 파일
# =============================================================================

# Step-1: 싱글스레드 CAN/모터 동작 확인
add_executable(test_can_init apps/test_can_init.cpp)
target_link_libraries(test_can_init PRIVATE qub_hw)

# 배선/채널 연결 확인용 안전 프로브 (non-actuating)
add_executable(test_connection apps/test_connection.cpp)
target_link_libraries(test_connection PRIVATE qub_hw)

# RobStride ID/영점 정비 CLI (non-RT, PCAN 4채널)
add_executable(motor_config apps/motor_config.cpp)
target_link_libraries(motor_config PRIVATE qub_hw ${PCAN_LIB} pthread rt)

# 자동 스캔 + 위치 모니터 + Space ON/OFF 터미널 GUI
add_executable(position_read apps/position_read.cpp)
target_link_libraries(position_read PRIVATE qub_hw ${PCAN_LIB} pthread rt)

# Step-2: 1채널 RT 모터 스레드
add_executable(test_rt_thread apps/test_rt_thread.cpp)
target_link_libraries(test_rt_thread PRIVATE qub_hw)

# Step-3: 4채널 동시 RT 모터 스레드
add_executable(test_4ch apps/test_4ch.cpp)
target_link_libraries(test_4ch PRIVATE qub_hw)

# Step-4: IMU 스레드 (mip_sdk 있을 때만)
if(MIP_SDK_FOUND)
    add_executable(test_imu apps/test_imu.cpp)
    target_link_libraries(test_imu PRIVATE qub_hw qub_imu)
endif()

# Step-5 이후 메인 컨트롤러
# add_executable(qub_controller apps/main.cpp)
# target_link_libraries(qub_controller PRIVATE qub_hw qub_imu onnxruntime yaml-cpp)

# =============================================================================
# 설치
# =============================================================================
install(TARGETS qub_hw ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/qub)
