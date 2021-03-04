cmake_minimum_required(VERSION 3.0)

project(RecastNavigation)

# lib versions
SET(SOVERSION 1)
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "5")
set(LIB_REVISION "1")
set(LIB_VERSION "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_REVISION}")
set(LIB_VERSION_NUM ${LIB_MAJOR_VERSION},${LIB_MINOR_VERSION},${LIB_REVISION},0)

option(RECASTNAVIGATION_DEMO "Build demo" ON)
option(RECASTNAVIGATION_TESTS "Build tests" ON)
option(RECASTNAVIGATION_EXAMPLES "Build examples" ON)

if(MSVC AND BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(GNUInstallDirs)

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
