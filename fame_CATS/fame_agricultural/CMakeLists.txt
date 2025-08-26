cmake_minimum_required(VERSION 3.5)
project(fame_agricultural)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(rclcpp REQUIRED)	
find_package(rclpy REQUIRED)

find_package(gazebo QUIET)	
    if(NOT gazebo_FOUND)	
        message(WARNING "Gazebo not found, proceeding without that simulator.")	
        return()	
    endif()	
 

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/robot_description")

foreach (INDEX RANGE 0 3)
    if (${INDEX} EQUAL 0)
        set(SUFFIX "")
        set(TOPIC_NS "Tractor")
    else ()
        set(SUFFIX "_${INDEX}")
        set(TOPIC_NS "Tractor${INDEX}")
    endif ()
    set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/robot_description/Tractor${SUFFIX}.sdf")
    message(STATUS "creating rules for ${URDF_FILE}")
    add_custom_command(
        OUTPUT ${URDF_FILE}
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/replace.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/robot_description/Tractor.sdf" "suffix=${SUFFIX}" "topic_ns=${TOPIC_NS}" ">" "${URDF_FILE}"
        DEPENDS robot_description/Tractor.sdf
        COMMENT "Generate ${URDF_FILE}"
        VERBATIM
    )
    add_custom_target(generate_urdf${SUFFIX} ALL DEPENDS ${URDF_FILE})
endforeach ()

foreach (INDEX RANGE 0 2)
    if (${INDEX} EQUAL 0)
        set(SUFFIX "")
        set(TOPIC_NS "Drone")
    else ()
        set(SUFFIX "_${INDEX}")
        set(TOPIC_NS "Drone${INDEX}")
    endif ()
    set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/robot_description/Drone${SUFFIX}.sdf")
    message(STATUS "creating rules for ${URDF_FILE}")
    add_custom_command(
        OUTPUT ${URDF_FILE}
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/replace.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/robot_description/Drone.sdf" "suffix=${SUFFIX}" "topic_ns=${TOPIC_NS}" ">" "${URDF_FILE}"
        DEPENDS robot_description/Drone.sdf
        COMMENT "Generate ${URDF_FILE}"
        VERBATIM
    )
    add_custom_target(generate_tello_urdf${SUFFIX} ALL DEPENDS ${URDF_FILE})
endforeach ()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)


install(
    DIRECTORY
        launch
        robot_description
        worlds
        "${CMAKE_CURRENT_BINARY_DIR}/robot_description"
    DESTINATION
        share/${PROJECT_NAME}/
)

install(
    PROGRAMS
        launch/multi_launch.py
        launch/spawn_elements.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
