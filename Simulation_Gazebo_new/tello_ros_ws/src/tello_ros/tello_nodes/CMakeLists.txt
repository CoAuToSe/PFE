cmake_minimum_required(VERSION 3.5)
project(tello_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate colcon
if ($ENV{CLION_IDE})
    message("Running inside CLion")
    set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tello_msgs REQUIRED)

#====== LAUNCH FILES ======#

##===== Install raw launch files =====#
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

##===== Install autocomplete launch files =====#

set(LAUNCH_FILES_EXTENSION "py" "xml" "yaml" "yml")

foreach(_ext LAUNCH_FILES_EXTENSION)
    # Collect source launch .${_ext} files
    file(GLOB _RAW_LAUNCH "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.${_ext}")

    # Where we generate properly-suffixed files
    set(_GEN_LAUNCH_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_launch")
    file(MAKE_DIRECTORY "${_GEN_LAUNCH_DIR}")

    set(_GEN_LAUNCH_OUTS)

    # IMPORTANT: iterate as a LIST
    foreach(_src IN LISTS _RAW_LAUNCH)
        get_filename_component(_name "${_src}" NAME)

        if(_name MATCHES "\\.launch\\.${_ext}$")
            set(_dst "${_GEN_LAUNCH_DIR}/${_name}")
        else()
            string(REGEX REPLACE "\\.${_ext}$" ".launch.${_ext}" _dst_name "${_name}")
            set(_dst "${_GEN_LAUNCH_DIR}/${_dst_name}")
        endif()

        # Generate real files (not symlinks) so install is robust
        add_custom_command(
            OUTPUT "${_dst}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_src}" "${_dst}"
            DEPENDS "${_src}"
            COMMENT "Generating ${_dst}"
            VERBATIM
        )
        list(APPEND _GEN_LAUNCH_OUTS "${_dst}")
    endforeach()

    # Ensure generation runs at build time
    add_custom_target(gen_launch ALL DEPENDS ${_GEN_LAUNCH_OUTS})

    # Install only the generated .launch.py files
    install(
        FILES ${_GEN_LAUNCH_OUTS}
        DESTINATION share/${PROJECT_NAME}/launch
    )
endforeach()

#====== END LAUNCH FILES ======#

ament_package()
