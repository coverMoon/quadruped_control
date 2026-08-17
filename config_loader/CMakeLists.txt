# 文件：CMakeLists.txt
# 作用：构建启动期 YAML 配置加载库 quadruped_config，源码目录与配置数据目录明确区分。

find_package(yaml-cpp REQUIRED)

add_library(quadruped_config
    src/controller_config_loader.cpp
    src/robot_model_loader.cpp
)

add_library(quadruped::config ALIAS quadruped_config)

target_include_directories(quadruped_config
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_config
    PUBLIC
        quadruped::core
    PRIVATE
        yaml-cpp
)

target_compile_features(quadruped_config PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_config PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_config PRIVATE /W4)
endif()
