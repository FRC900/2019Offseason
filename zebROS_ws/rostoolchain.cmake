cmake_minimum_required(VERSION 2.8)
set(CMAKE_SYSTEM_NAME Linux)
set(ARM_PREFIX arm-frc2019-linux-gnueabi)

set(CMAKE_C_COMPILER ${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_PREFIX}-g++)

set(CMAKE_SYSROOT /home/ubuntu/frc2019/roborio/${ARM_PREFIX})

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT};$ENV{HOME}/2018Offseason/zebROS_ws/install_isolated)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(BOOST_ROOT ${ARM_PREFIX})
set(Boost_NO_SYSTEM_PATHS=ON)

add_definitions(-std=c++11)

find_program(CMAKE_RANLIB ${ARM_PREFIX}-gcc-ranlib)
find_program(CMAKE_AR ${ARM_PREFIX}-gcc-ar)
set(OPT_FLAGS "-O3 -flto=4 -mcpu=cortex-a9 -mfpu=neon -fvect-cost-model")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_INSTALL_RPATH "$ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/kinetic/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
