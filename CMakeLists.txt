cmake_minimum_required(VERSION 3.10)

project(rm_auto_aim_mvp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成 compile_commands.json，给 VS Code IntelliSense 用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

add_executable(auto_aim
    src/main.cpp
    src/detector.cpp
    src/pnp_solver.cpp
    src/coord_transform.cpp
)

target_include_directories(auto_aim PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(auto_aim PRIVATE
    ${OpenCV_LIBS}
)