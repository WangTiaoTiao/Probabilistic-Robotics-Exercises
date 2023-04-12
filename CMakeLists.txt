cmake_minimum_required(VERSION 3.24.3)

project(P_ROBOTICS VERSION 1.0.0)

# 设置编译选项
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#Dependencies
#############################


# 添加编译目标
add_executable(ch02 src/exercise_02.cpp)
add_executable(ch03 src/exercise_03.cpp)

# 添加库文件
# add_library(utils utils.cpp)
# target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(main utils)