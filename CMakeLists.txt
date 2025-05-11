cmake_minimum_required(VERSION 3.10.2)
project("openglandroid")

add_library(native-lib SHARED app/src/main/cpp/native-lib.cpp)

find_library(log-lib   log)
find_library(android-lib android)
find_library(egl-lib   EGL)
find_library(gles-lib  GLESv3)

target_link_libraries(native-lib
    ${log-lib}
    ${android-lib}
    ${egl-lib}
    ${gles-lib}
)
