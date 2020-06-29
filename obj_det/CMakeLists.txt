# ----------------------------------------------------------------------------
#  Root CMake file for the DWM1001 Project
# ----------------------------------------------------------------------------
#
# To perform an out of source build do the following on the command line:
# 1. navigate to the root project directory
# 2. mkdir build
# 3. cd build
# 4. depending on your system (windows/linux) type the following
# Example CMake build line for windows & VS2015
# windows: cmake -G "Visual Studio 14 2015 Win64" -DUSE_AVX_INSTRUCTIONS=ON ..
# linux: cmake -DUSE_AVX_INSTRUCTIONS=ON ..
#
# 5. To compile:
# windows: cmake --build . --config Release
# linux: cmake --build . --config Release -- -j4
#

cmake_minimum_required(VERSION 2.8.12)

PROJECT(object_detector)
set(exe_name "obj_det_ex")

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# use this option to turn compilation with ROS off for Windows and non-ROS linux testing
option(USE_ROS "Compile the program with ROS" ON)

# This looks for an environment variable called PLATFORM.  This can be used to point
# cmake to various locations based on the individual machines that the code is complied on
set(PLATFORM "$ENV{PLATFORM}" CACHE INTERNAL "Copied from environment variable")
message("Platform: " ${PLATFORM})

if(${PLATFORM} STREQUAL "Laptop")
    set(COMMON_ROOT "D:/Common")
    set(DLIB_CONTRIB "D:/Projects/dlib-contrib")
    option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  ON)
    option(DLIB_USE_CUDA "" ON)
    include(D:/dlib-19.19/dlib/cmake)
elseif(${PLATFORM} STREQUAL "MainGear")
    set(_ECLIPSE_VERSION "3.8")
    set(COMMON_ROOT "~/Projects/Common")
    set(DLIB_CONTRIB "~/Projects/dlib-contrib")
    option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  ON)
    option(DLIB_USE_CUDA "" ON)
    include(~/dlib-19.19/dlib/cmake)
elseif(${PLATFORM} STREQUAL "DLIB_DOCKER")
    set(COMMON_ROOT "~/Projects/Common")
    set(DLIB_CONTRIB "~/Projects/dlib-contrib")
    option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  ON)
    option(DLIB_USE_CUDA "" ON)
    include(~/dlib-19.19/dlib/cmake)
elseif(${PLATFORM} STREQUAL "JAX")
    set(COMMON_ROOT "~/Projects/Common")
    set(DLIB_CONTRIB "~/Projects/dlib-contrib")
    option(DLIB_USE_CUDA "" ON)
    include(~/dlib-19.19/dlib/cmake)
else()
    set(COMMON_ROOT "~/Projects/Common")
    set(DLIB_CONTRIB "~/Projects/dlib-contrib")
    option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  ON)
    option(DLIB_USE_CUDA "" ON)
    include(~/dlib-19.19/dlib/cmake)
endif()

#include(${COMMON_ROOT}/cmake_utils/use_cpp_11.cmake)
#include(${COMMON_ROOT}/cmake_utils/use_sse_avx.cmake)

set(project_includes "include" 
    "${COMMON_ROOT}/include"
    "${DLIB_CONTRIB}/include" 
    )

if (${USE_ROS})
    message("")
    message("Compiling with ROS!")
    list(APPEND active_preprocessor_switches "-DUSE_ROS")
endif()

if(UNIX)
    if(USE_ROS)

        find_package(catkin 
            COMPONENTS
            roscpp
            rosconsole
            std_msgs
            sensor_msgs
#            nav_msgs
#            geometry_msgs
#            dynamic_reconfigure
#            roslint
            message_generation
        )

        add_message_files(
            FILES
            object_det.msg
            object_det_list.msg
        )

        generate_messages(DEPENDENCIES
            std_msgs
    	    sensor_msgs
#            nav_msgs
#            geometry_msgs
        )

        catkin_package(
            CATKIN_DEPENDS
            roscpp
            rosconsole
            std_msgs
            sensor_msgs
#            nav_msgs
#            geometry_msgs
#            dynamic_reconfigure
#            stereo_msgs
            message_runtime
        )

        set(project_includes ${project_includes} ${catkin_INCLUDE_DIRS})

        add_definitions(-std=c++11)
        set(LINK_LIBRARIES ${catkin_LIBRARIES})

    endif()
endif()

# look for any *.c and *.cpp files
file(GLOB SOURCES "src/*.c*"
    "${COMMON_ROOT}/src/*.cpp"
    "${DLIB_CONTRIB}/src/*.cpp"
    )

message(" ")
message("--------------------------------------------------------")
message("Source Files: ")
foreach(S ${SOURCES})
    message("  " ${S})
endforeach()
message("--------------------------------------------------------")

message(" ")

message("--------------------------------------------------------")
message("Project Includes: ")
foreach(S ${project_includes})
    message("  " ${S})
endforeach()
message("--------------------------------------------------------")

message(" ")

message("--------------------------------------------------------")
message("Preprocessor Switches: ")
foreach(S ${active_preprocessor_switches})
message("  " ${S})
endforeach()
message("--------------------------------------------------------")
message(" ")


if(WIN32)
    include_directories(${project_includes})
    add_executable(${exe_name} ${SOURCES})
    target_link_libraries(${exe_name} dlib::dlib)

    message("MSVC Version: " ${MSVC_VERSION})

    set(default_build_type "Release")
    if(MSVC AND (${CMAKE_VERSION} VERSION_GREATER "3.6.0"))
        target_compile_options(${exe_name} PRIVATE "/MP" ${active_preprocessor_switches})
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${exe_name})
    endif()

elseif(UNIX)

if(USE_ROS)
    include_directories(${project_includes})

    add_executable(${exe_name} ${SOURCES})
    target_link_libraries(${exe_name} dlib::dlib ${LINK_LIBRARIES})
    add_dependencies(${exe_name} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencfg)
    target_compile_options(${exe_name} PRIVATE ${active_preprocessor_switches})

    message("")
#    message("catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")
#    roslint_cpp()

###############################################################################
# INSTALL

    install(TARGETS
        ${exe_name}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

    #install(FILES
    #    nodelet_plugins.xml
    #    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    #)

    install(DIRECTORY
        launch
        urdf
        params
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

else()
    include_directories(${project_includes})
    add_executable(${exe_name} ${SOURCES})
    target_link_libraries(${exe_name} dlib::dlib)

endif()

endif()

message(" ")
message(STATUS "Build Complete!")
message(" ")

