cmake_minimum_required(VERSION 3.10.2)
project("OpenGLAndroid")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 1) Figure out your NDK host tag
if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
  set(ANDROID_HOST "windows-x86_64")
elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
  set(ANDROID_HOST "linux-x86_64")
elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
  set(ANDROID_HOST "darwin-x86_64")
endif()

# 2) Declare the native-lib target
add_library(native-lib SHARED
    app/src/main/cpp/native-lib.cpp
    app/src/main/cpp/input/touch_input.cpp
    app/src/main/cpp/model/obj_loader.cpp
    app/src/main/cpp/model/model.cpp
    app/src/main/cpp/renderer/mesh.cpp
    app/src/main/cpp/renderer/shader.cpp
    app/src/main/cpp/core/scene/lights/point_light.cpp
    app/src/main/cpp/core/scene/lights/directional_light.cpp
    app/src/main/cpp/core/scene/lights/spot_light.cpp
    app/src/main/cpp/renderer/texture_utils.cpp


)

# 4) Your normal include dirs
target_include_directories(native-lib PRIVATE
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/core/scene/entities
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/core/scene/lights
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/core/scene
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/core/utils
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/external
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/model
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/renderer
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/input
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/scene
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/utils
)

# 5) Find & link Android libs
find_library(log-lib     log)
find_library(android-lib android)
find_library(egl-lib     EGL)
find_library(gles-lib    GLESv3)

target_link_libraries(native-lib
    ${log-lib}
    ${android-lib}
    ${egl-lib}
    ${gles-lib}
)
