cmake_minimum_required(VERSION 3.0.2) #对于cmake版本的最低版本的要求
project(OBS_DETECT_MODULE)#建立的工程名，例如源代码目录路径的变量名为CH_DIR
                #工程存储目录变量名为CH_BINARY_DIR
#要求工程依赖的PCL最低版本为1.3，并且版本至少包含common和IO两个模块  这里的REQUIRED意味着如果对应的库找不到 则CMake配置的过程将完全失败，
#因为PCL是模块化的，也可以如下操作：
#           一个组件  find_package(PCL 1.6 REQUIRED COMPONENTS  io)
#           多个组件  find_package(PCL 1.6 REQUIRED COMPONENTS commom io)
#           所有组件  find_package(PCL 1.6 REQUIRED ) 
include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

#如果设置为"Release"模式，设置断点后在调试时是默认无效的，进入调试后，断点会变为空心断点，程序不会在任何断点处停下
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS "-O3")#一定要注释掉该句（或不写），否则“任何级别的优化都将带来代码结构的改变，将会使目标代码的执行顺序变得面目全非，导致调试信息严重不足
include(CPack)

add_compile_options(-std=c++11)


find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)


# aux_source_directory(
#     src/humandetect _SOURCES
# )
# 自动扫描指定路径下的源文件，并将它们添加到变量 _SUB_SOURCES 中
aux_source_directory(
    src/humandetect/src _SUB_SOURCES
)
aux_source_directory(
    src/humandetect/common _COMMON_SOURCES
)

# 设置头文件的搜索路径 gcc -I
include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src/humandetect/common
  src/humandetect/include
  src/humandetect/include/openni
  src/humandetect/include/eigen-3.4.0

  src/readradar
)


# 设置目标库文件的搜索路径 gcc -L
link_directories(
  /usr/lib
  /usr/local/lib
  /home/nvidia/AstraPro_SDK_OpenNI/samples/samples/ThirdParty/OpenNI2/arm/Arm64   #openni
  ${PCL_LIBRARIES_DIRS}
  ${OpenCV_LIBRARIES_DIRS}
  ${CUDA_LIBRARIES_DIRS}
)
add_definitions(${PCL_DEFINITIONS})


#message("***$$include folder: "${catkin_INCLUDE_DIRS})

#这句话告诉CMake从单个源文件pcd_capture建立一个可执行文件
add_executable(environment_wj src/environment.cpp
src/readradar/ord_lidar_driver.cpp src/readradar/radar_read.cpp
src/render/render.cpp
src/DWA/dwa_module.cpp
src/DWA/trans_module.cpp
src/DWA/simple_pathplaning.cpp
src/serialsend/serialsend.cpp src/serialsend/uart-drv-linux.cpp src/serialsend/uart.c 
src/readimu/readimu.cpp src/readimu/imu_data_decode.c src/readimu/packet.c src/readimu/ch_serial.c
src/blindcompensate/blindcompensate.cpp
src/occupymap/occupymapsimple.cpp
src/datasave/datasave.cpp
src/humandetect/humandetect.cpp ${_SUB_SOURCES} ${_COMMON_SOURCES}
)#后面的文件地址是以CmakeLists.txt所在文件夹为根目录出发寻找的


## Specify libraries to link a library or executable target against
#虽然包含了PCL的头文件，因此编译器知道我们现在访问所用的方法，我们也需要让链接器知道所链接的库
target_link_libraries(environment_wj
  ${LIBS}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES}
  # cudart
  cudnn
  # cublas
  nvinfer
  nvinfer_plugin
  nvparsers
  nvonnxparser
  OpenNI2

  Threads::Threads
  oradar_sdk          # core
)

message(${PCL_LIBRARIES})
message(${PCL_LIBRARIES})

# target_link_libraries(multi_cam
#   ${LIBS}
#   ${CUDA_LIBRARIES}
#   ${OpenCV_LIBS} 
#   ${realsense2_LIBRARY}
#   ${PCL_LIBRARIES}
#   cudart
#   cudnn
#   cublas
#   nvinfer
#   nvinfer_plugin
#   nvparsers
#   nvonnxparser
#   OpenNI2
# )

# target_link_libraries(get3dcoordtest
#   ${OpenCV_LIBS} 
#   ${realsense2_LIBRARY}
#   ${PCL_LIBRARIES}
# )

# target_link_libraries(dwatest
#   ${OpenCV_LIBS} 
#   ${realsense2_LIBRARY}
#   ${PCL_LIBRARIES}
# )


