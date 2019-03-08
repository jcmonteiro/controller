cmake_minimum_required(VERSION 2.8)
project(pid-control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(LIBNAME "pid_control")

find_package(linear_system REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("include" ${EIGEN3_INCLUDE_DIRS})
add_library(${LIBNAME} SHARED "src/PID.cpp")
target_link_libraries(${LIBNAME} linear_system)

# Install
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "include/PID.hpp;include/ParallelPIDSettings.hpp")
install(
    TARGETS ${LIBNAME}
    EXPORT "${LIBNAME}-export"
    LIBRARY DESTINATION "lib/${LIBNAME}"
    ARCHIVE DESTINATION "lib/${LIBNAME}"
    PUBLIC_HEADER DESTINATION "include/${LIBNAME}"
)
install(
    EXPORT "${LIBNAME}-export"
    FILE "${LIBNAME}-config.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/${LIBNAME}/cmake"
)
