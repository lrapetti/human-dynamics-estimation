project(human-robot-pose LANGUAGES CXX VERSION 0.1)
# See https://github.com/robotology/how-to-export-cpp-library/blob/master/CMakeLists.txt for details.

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

find_package(YCM REQUIRED)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

# CONFIGURE THE TARGET
# ====================

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HumanRobotPose.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
set(HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/HumanRobotPose.h)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    YARP::YARP_OS
    YARP::YARP_init
    YARP::YARP_sig
    YARP::YARP_dev
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-high-level
    human-state-provider-data
    human-rosmsgs)

if (VERBOSE_BUILD)
  add_warnings_configuration_to_target(TARGETS human-jointstate-bridge PRIVATE)
endif()

# INSTALLATION
# ============

yarp_install(FILES conf/human-robot-pose.ini
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/human-dynamic-estimation)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})

