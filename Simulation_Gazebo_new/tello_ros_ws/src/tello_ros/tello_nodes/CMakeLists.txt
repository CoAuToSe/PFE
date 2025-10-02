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

# ##===== Install raw launch files =====#
# install(
#     DIRECTORY launch/
#     DESTINATION share/${PROJECT_NAME}/launch
# )

##===== Install autocomplete launch files =====#
# --- Collect raw launch sources in launch/ (4 formats) ---
file(GLOB _RAW_LAUNCH_PY   "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.py")
file(GLOB _RAW_LAUNCH_XML  "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.xml")
file(GLOB _RAW_LAUNCH_YAML "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.yaml" "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.yml")
set(_RAW_LAUNCH ${_RAW_LAUNCH_PY} ${_RAW_LAUNCH_XML} ${_RAW_LAUNCH_YAML})

# Where we produce properly-suffixed launch files
set(_GEN_LAUNCH_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_launch")
file(MAKE_DIRECTORY "${_GEN_LAUNCH_DIR}")

set(_GEN_LAUNCH_OUTS)

foreach(_src IN LISTS _RAW_LAUNCH)
    get_filename_component(_name "${_src}" NAME)

    # If it's already a canonical launch filename, keep it
    if(_name MATCHES "\\.launch\\.(py|xml|ya?ml)$")
        set(_dst_name "${_name}")
    else()
        if(_name MATCHES "\\.py$")
            string(REGEX REPLACE "\\.py$" ".launch.py" _dst_name "${_name}")
        elseif(_name MATCHES "\\.xml$")
            string(REGEX REPLACE "\\.xml$" ".launch.xml" _dst_name "${_name}")
        elseif(_name MATCHES "\\.ya?ml$")
            # normalize .yaml/.yml -> .launch.yaml
            string(REGEX REPLACE "\\.ya?ml$" ".launch.yaml" _dst_name "${_name}")
        else()
            # Skip anything unexpected
            message(WARNING "Skipping non-launch file: ${_name}")
            continue()
        endif()
    endif()

    set(_dst "${_GEN_LAUNCH_DIR}/${_dst_name}")

    # Generate real files (portable; avoids nested symlink issues)
    add_custom_command(
        OUTPUT  "${_dst}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_src}" "${_dst}"
        DEPENDS "${_src}"
        COMMENT "Generating ${_dst_name}"
        VERBATIM
    )
    list(APPEND _GEN_LAUNCH_OUTS "${_dst}")
endforeach()

# Ensure generation happens during the build
add_custom_target(gen_launch ALL DEPENDS ${_GEN_LAUNCH_OUTS})

# Install only the generated, canonical launch files
install(
    FILES ${_GEN_LAUNCH_OUTS}
    DESTINATION share/${PROJECT_NAME}/launch
)

# (Optional) Also install the rest of launch/ but avoid double-installing
# the generated launch files themselves.
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    PATTERN "*.launch.py"   EXCLUDE
    PATTERN "*.launch.xml"  EXCLUDE
    PATTERN "*.launch.yaml" EXCLUDE
)

#====== END LAUNCH FILES ======#

ament_package()
