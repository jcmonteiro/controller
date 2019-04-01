cmake_minimum_required(VERSION 3.0)
project(controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(LIBNAME "controller")

if(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES Debug)
    # debug mode stuff here
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")
else()
    # release mode stuff here
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

use_cxx11()

find_package(Boost COMPONENTS unit_test_framework)
find_package(linear_system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 QUIET)

include_directories("include" ${EIGEN3_INCLUDE_DIRS})
set(LIBRARY_SOURCES
    src/Controller.cpp
    src/FilteredController.cpp
    src/PID.cpp
    src/SettingsPID.cpp
    src/SettingsPPI.cpp
)
add_library(${LIBNAME} SHARED "${LIBRARY_SOURCES}")
target_link_libraries(${LIBNAME} linear_system)

# Python bindings
if (pybind11_FOUND)
    pybind11_add_module(${LIBNAME}_py python/python_bindings.cpp)
    target_link_libraries(${LIBNAME}_py PUBLIC ${LIBNAME})
    set(BUILD_PYTHON_DIR "${CMAKE_CURRENT_BINARY_DIR}/${LIBNAME}")
    set_target_properties(${LIBNAME}_py
        PROPERTIES
        SUFFIX ".so"
        LIBRARY_OUTPUT_DIRECTORY "${BUILD_PYTHON_DIR}"
    )
    configure_file(
        "python/__init__.py"
        "${BUILD_PYTHON_DIR}/__init__.py"
    )
    configure_file(
        "python/setup.py"
        "${BUILD_PYTHON_DIR}/setup.py"
    )
endif ()

# Unit tests
if (Boost_FOUND)
    ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
    add_executable(test-library "test/test_PID.cpp")
    target_include_directories(test-library PRIVATE ${Boost_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
    target_link_libraries(test-library ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LIBNAME})
    add_test(NAME test-1 COMMAND test-library)
endif ()

# Install
set(LIBRARY_HEADERS
    include/Controller.hpp
    include/FilteredController.hpp
    include/PID.hpp
    include/SettingsPID.hpp
    include/SettingsPPI.hpp
)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "${LIBRARY_HEADERS}")
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

# Install python library
if (pybind11_FOUND)
    install(CODE "execute_process(COMMAND python3 setup.py install
        --prefix=${CMAKE_INSTALL_PREFIX} WORKING_DIRECTORY ${BUILD_PYTHON_DIR})")
endif ()
