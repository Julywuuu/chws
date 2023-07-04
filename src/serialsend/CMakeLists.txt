cmake_minimum_required(VERSION 2.8)
PROJECT(serialsend)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# 头文件 需要把 ord_lidar_driver.h + radar_read.h + core 都引入
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
)


# 生成可执行文件
add_executable(main 
    main.cpp
    serialsend.cpp
    uart-drv-linux.cpp
    uart.c
)





