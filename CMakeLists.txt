cmake_minimum_required(VERSION 3.0)

project(RecastNavigation)

# lib versions
SET(SOVERSION 1)
set(LIB_VERSION 1.5.1)
string(REPLACE "." "," LIB_VERSION_NUM "${LIB_VERSION}.0")

option(RECASTNAVIGATION_DEMO "Build demo" ON)
option(RECASTNAVIGATION_TESTS "Build tests" ON)
option(RECASTNAVIGATION_EXAMPLES "Build examples" ON)

if(MSVC AND BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(GNUInstallDirs)

configure_file(
    "${RecastNavigation_SOURCE_DIR}/config.h.in"
    "${RecastNavigation_BINARY_DIR}/config.h")
include_directories( ${PROJECT_BINARY_DIR} )
SET(RC_MAX_LAYERS 63 CACHE STRING "Must be 255 or smaller (not 256) because layer IDs are stored as a byte where 255 is a special value.")
SET(RC_MAX_NEIS 16 CACHE STRING "" FORCE)

if(${RC_MAX_LAYERS} GREATER 255)
	message(FATAL_ERROR "RC_MAX_LAYERS must be <= 255")
endif()

configure_file(
    "${RecastNavigation_SOURCE_DIR}/version.h.in"
    "${RecastNavigation_BINARY_DIR}/version.h")
install(FILES "${RecastNavigation_BINARY_DIR}/version.h" DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)

# Needed for recastnavigation.pc.in
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
set(libdir "\${exec_prefix}/${CMAKE_INSTALL_LIBDIR}")
set(bindir "\${exec_prefix}/${CMAKE_INSTALL_BINDIR}")
set(includedir "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
set(PACKAGE_VERSION "${LIB_VERSION}")
configure_file(
        "${RecastNavigation_SOURCE_DIR}/recastnavigation.pc.in"
        "${RecastNavigation_BINARY_DIR}/recastnavigation.pc"
        @ONLY)
install(FILES "${RecastNavigation_BINARY_DIR}/recastnavigation.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

add_subdirectory(DebugUtils)
add_subdirectory(Detour)
add_subdirectory(DetourCrowd)
add_subdirectory(DetourTileCache)
add_subdirectory(Recast)

if (RECASTNAVIGATION_DEMO)
    add_subdirectory(RecastDemo)
endif ()

if (RECASTNAVIGATION_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif ()
