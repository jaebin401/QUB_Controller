cmake_minimum_required(VERSION 3.16)
project(QUB_Controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# 빌드 타입 기본값
# =============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g -Wall -Wextra -fsanitize=address")

add_compile_definitions(_GNU_SOURCE)

# =============================================================================
# PCANBasic 라이브러리 찾기
# =============================================================================
find_path(PCAN_INCLUDE_DIR
    NAMES PCANBasic.h
    PATHS
        /usr/include
        /usr/local/include
        ${PCAN_INCLUDE_DIR}
)

find_library(PCAN_LIB
    NAMES pcanbasic
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${PCAN_LIB_DIR}
)

if(NOT PCAN_INCLUDE_DIR OR NOT PCAN_LIB)
    message(FATAL_ERROR
        "PCANBasic 헤더/라이브러리를 찾지 못했습니다.\n"
        "peak-linux-driver 를 먼저 설치하거나 아래 옵션을 지정하세요:\n"
        "  cmake -DPCAN_INCLUDE_DIR=<path> -DPCAN_LIB_DIR=<path> .")
endif()

message(STATUS "PCAN include: ${PCAN_INCLUDE_DIR}")
message(STATUS "PCAN lib:     ${PCAN_LIB}")

# =============================================================================
# ONNX Runtime (Policy thread 에서 사용 - 아직 미구현)
# =============================================================================
if(DEFINED ONNXRUNTIME_ROOT)
    set(ORT_INCLUDE "${ONNXRUNTIME_ROOT}/include")
    set(ORT_LIB    "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_ROOT}")
else()
    message(STATUS "ONNX Runtime: 미지정 (policy_runner 빌드 제외)")
endif()

# =============================================================================
# 핵심 라이브러리: qub_hw
# =============================================================================
add_library(qub_hw STATIC
    src/pcan_channel.cpp
    src/robstride.cpp
    src/motor_thread.cpp
    # src/imu_driver.cpp        # Step-4
    # src/policy_runner.cpp     # Step-5
)

target_include_directories(qub_hw PUBLIC
    include
    ${PCAN_INCLUDE_DIR}
)

target_link_libraries(qub_hw PUBLIC
    ${PCAN_LIB}
    pthread
    rt
)

# =============================================================================
# 실행 파일들
# =============================================================================

# -- Step-1: 싱글스레드 CAN/모터 동작 확인 ---------------------------------
add_executable(test_can_init apps/test_can_init.cpp)
target_link_libraries(test_can_init PRIVATE qub_hw)

# -- Step-2: 1채널 RT 모터 스레드 + 루프 타이밍 ----------------------------
add_executable(test_rt_thread apps/test_rt_thread.cpp)
target_link_libraries(test_rt_thread PRIVATE qub_hw)

# -- Step-3: 4채널 동시 RT 모터 스레드 -------------------------------------
add_executable(test_4ch apps/test_4ch.cpp)
target_link_libraries(test_4ch PRIVATE qub_hw)

# -- 메인 컨트롤러 (Step-5 이후) ------------------------------------------
# add_executable(qub_controller apps/main.cpp)
# target_link_libraries(qub_controller PRIVATE qub_hw onnxruntime yaml-cpp)

# =============================================================================
# 설치
# =============================================================================
install(TARGETS qub_hw ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/qub)