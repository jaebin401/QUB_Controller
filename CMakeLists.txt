cmake_minimum_required(VERSION 3.16)
project(QUB_Controller CXX)

set(CMAKE_CXX_STANDARD 17)
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
        PATHS "${MIP_SDK_ROOT}/include" "${MIP_SDK_ROOT}/src/cpp"
        NO_DEFAULT_PATH
    )
    find_library(MIP_LIB
        NAMES mip microstrain_inertial_mip
        PATHS "${MIP_SDK_ROOT}/lib" "${MIP_SDK_ROOT}/build/src/cpp"
        NO_DEFAULT_PATH
    )
    if(MIP_INCLUDE_DIR AND MIP_LIB)
        set(MIP_SDK_FOUND TRUE)
        message(STATUS "mip_sdk include: ${MIP_INCLUDE_DIR}")
        message(STATUS "mip_sdk lib:     ${MIP_LIB}")
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
    target_include_directories(qub_imu PUBLIC include ${MIP_INCLUDE_DIR})
    target_link_libraries(qub_imu PUBLIC ${MIP_LIB} pthread rt)
endif()

# =============================================================================
# 실행 파일
# =============================================================================

# Step-1: 싱글스레드 CAN/모터 동작 확인
add_executable(test_can_init apps/test_can_init.cpp)
target_link_libraries(test_can_init PRIVATE qub_hw)

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
