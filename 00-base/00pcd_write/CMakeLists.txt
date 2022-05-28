cmake_minimum_required(VERSION 3.8 FATAL_ERROR) # #对于cmake版本的最低版本的要求

project(pcd_write) #  建立的工程名，例如源代码目录路径的变量名为CH_DIR

#因为PCL是模块化的，也可以如下操作：
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write pcd_write.cpp)

target_link_libraries (pcd_write ${PCL_LIBRARIES}) # 