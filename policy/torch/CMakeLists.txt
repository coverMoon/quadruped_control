# 文件：policy/torch/CMakeLists.txt
# 作用：构建仅在 RL profile 启用的 LibTorch TorchScript runner。

add_library(quadruped_torch_policy
    src/torch_policy.cpp
)

add_library(quadruped::torch_policy ALIAS quadruped_torch_policy)

target_include_directories(quadruped_torch_policy
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_torch_policy
    PUBLIC
        quadruped::motion
    PRIVATE
        ${TORCH_LIBRARIES}
)

target_compile_features(quadruped_torch_policy PUBLIC cxx_std_17)
target_compile_options(quadruped_torch_policy PRIVATE ${TORCH_CXX_FLAGS})
target_include_directories(quadruped_torch_policy PRIVATE ${TORCH_INCLUDE_DIRS})
