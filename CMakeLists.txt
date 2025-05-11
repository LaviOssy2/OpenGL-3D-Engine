cmake_minimum_required(VERSION 3.10.2)
project("openglandroid")

add_library(native-lib SHARED
    app/src/main/cpp/native-lib.cpp
    app/src/main/cpp/Model.h
    app/src/main/cpp/Mesh.h
    app/src/main/cpp/Shader.h
)

# Point to your vcpkg glm
target_include_directories(native-lib PRIVATE
    ${CMAKE_SOURCE_DIR}/app/src/main/cpp
    C:/MyApps/dev/vcpkg-master/vcpkg-master/installed/x64-windows/include
)

find_library(log-lib log)
find_library(android-lib android)
find_library(egl-lib EGL)
find_library(gles-lib GLESv3)

target_link_libraries(native-lib
    ${log-lib}
    ${android-lib}
    ${egl-lib}
    ${gles-lib}
)
