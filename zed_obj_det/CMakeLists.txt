# ----------------------------------------------------------------------------
#  Root CMake file
# ----------------------------------------------------------------------------
#
# To perform an out of source build do the following on the command line:
# 1. navigate to the root project directory (in this case DfD2)
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

PROJECT(zed_obj_det)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# use this option to turn compilation with ROS off for Windows and non-ROS linux testing
option(USE_ROS "Compile the program with ROS" ON)

set(project_includes
    "include"
    )

if (${USE_ROS})
    message("")
    message("Compiling with ROS!")
    list(APPEND active_preprocessor_switches "-DUSE_ROS")

endif()

if(UNIX)
    if(USE_ROS)

        find_package(catkin COMPONENTS
            #roscpp
            #rosconsole
            std_msgs
            dynamic_reconfigure
            roslint
            message_generation
        )

        add_message_files( 
            FILES
            object_det.msg
            object_det_list.msg
            )

        generate_messages(DEPENDENCIES
            std_msgs
            )

        catkin_package(
            CATKIN_DEPENDS
            #roscpp
            #rosconsole
            std_msgs
            message_runtime
            )

        set(project_includes ${project_includes} ${catkin_INCLUDE_DIRS})

        add_definitions(-std=c++11)
        set(LINK_LIBRARIES ${catkin_LIBRARIES})

    endif()
endif()

# look for any *.c and *.cpp files
file(GLOB SOURCES "src/*.c*"
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


if(USE_ROS)
    include_directories(${project_includes})


###############################################################################
# INSTALL

    install(TARGETS
        ${exe_name}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

    install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

else()
    include_directories(${project_includes})
    add_executable(${exe_name} ${SOURCES})

endif()


message(" ")
message(STATUS "Build Complete!")
message(" ")

