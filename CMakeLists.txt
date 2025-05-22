cmake_minimum_required(VERSION 3.10.2)
project("OpenGLAndroid")

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
    app/src/main/cpp/model/obj_loader.cpp
    app/src/main/cpp/renderer/model.cpp
    app/src/main/cpp/renderer/mesh.cpp
    app/src/main/cpp/renderer/renderer.cpp
    app/src/main/cpp/utils/asset_loader.h
)

# 4) Your normal include dirs
target_include_directories(native-lib PRIVATE
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/external
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/model
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/renderer
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/utils
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp/core
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
