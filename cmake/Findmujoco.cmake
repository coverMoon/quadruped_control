# 文件：Findmujoco.cmake
# 作用：查找固定安装的 MuJoCo C/C++ 头文件和动态库。

get_filename_component(
    _MUJOCO_DEFAULT_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../.deps/mujoco-3.9.0"
    ABSOLUTE
)

set(
    mujoco_ROOT
    "${_MUJOCO_DEFAULT_ROOT}"
    CACHE PATH
    "MuJoCo 发行包根目录"
)

find_path(
    mujoco_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${mujoco_ROOT}/include"
    NO_DEFAULT_PATH
)

find_library(
    mujoco_LIBRARY
    NAMES mujoco
    HINTS "${mujoco_ROOT}/lib"
    NO_DEFAULT_PATH
)

if(mujoco_INCLUDE_DIR)
    file(
        STRINGS
        "${mujoco_INCLUDE_DIR}/mujoco/mujoco.h"
        mujoco_VERSION_LINE
        REGEX "^#define mjVERSION_HEADER [0-9]+$"
    )
    string(REGEX MATCH "[0-9]+$" mujoco_VERSION_NUMBER "${mujoco_VERSION_LINE}")

    if(mujoco_VERSION_NUMBER)
        math(EXPR mujoco_VERSION_MAJOR "${mujoco_VERSION_NUMBER} / 1000000")
        math(EXPR mujoco_VERSION_MINOR "(${mujoco_VERSION_NUMBER} / 1000) % 1000")
        math(EXPR mujoco_VERSION_PATCH "${mujoco_VERSION_NUMBER} % 1000")
        set(
            mujoco_VERSION
            "${mujoco_VERSION_MAJOR}.${mujoco_VERSION_MINOR}.${mujoco_VERSION_PATCH}"
        )
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    mujoco
    REQUIRED_VARS mujoco_LIBRARY mujoco_INCLUDE_DIR
    VERSION_VAR mujoco_VERSION
)

if(mujoco_FOUND AND NOT TARGET mujoco::mujoco)
    add_library(mujoco::mujoco SHARED IMPORTED)
    set_target_properties(
        mujoco::mujoco
        PROPERTIES
            IMPORTED_LOCATION "${mujoco_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARY)

unset(_MUJOCO_DEFAULT_ROOT)
