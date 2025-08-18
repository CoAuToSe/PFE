cmake_minimum_required(VERSION 3.5)
project(tello_description)

find_package(ament_cmake REQUIRED)

# Env
# set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
# set(OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/urdf")

#=============
# Generate N urdf files, each with unique robot and frame names
# Source file is in ${CMAKE_CURRENT_SOURCE_DIR}/urdf/tello.xml
# Generated files are in ${CMAKE_CURRENT_BINARY_DIR}/urdf/telloX.urdf
#=============

# Crée le dossier de sortie si inexistant
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf")


# Récupère tous les fichiers du dossier source  
file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*")


foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(ROBOT_FILE_NAME ${SRC_FILE} NAME_WLE)
    # get_filename_component(FILE_EXT ${SRC_FILE} LAST_EXT)
    foreach (INDEX RANGE 0 8)
        if (${INDEX} EQUAL 0)
            set(ROBOT_SUFFIX "")
            set(TOPIC_NS "${ROBOT_FILE_NAME}")
        else ()
            set(ROBOT_SUFFIX "_${INDEX}")
            set(TOPIC_NS "${ROBOT_FILE_NAME}_${INDEX}")
        endif ()
        set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/urdf/${ROBOT_FILE_NAME}${ROBOT_SUFFIX}.urdf")
        message(STATUS "creating rules for ${URDF_FILE}")
        add_custom_command( OUTPUT ${URDF_FILE}
            COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/replace.py"
                "${CMAKE_CURRENT_SOURCE_DIR}/urdf/tello.xml" "suffix=${ROBOT_SUFFIX}" "topic_ns=${TOPIC_NS}" "robot_name=${ROBOT_FILE_NAME}" ">" "${URDF_FILE}"
            DEPENDS ${SRC_FILE}
            COMMENT "Generating ${URDF_FILE} from ${SRC_FILE}"
            VERBATIM
        )
        # TODO investigate why configure_file isn't working
        # configure_file("urdf/tello.xml" ${URDF_FILE})
        add_custom_target(generate_urdf${ROBOT_SUFFIX} ALL DEPENDS ${URDF_FILE})
    endforeach ()
    list(APPEND GENERATED_FILES ${OUT_FILE})
endforeach()
#=============
# Install
#=============

install(
    DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf"
    DESTINATION share/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
