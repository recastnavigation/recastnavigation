cmake_minimum_required(VERSION 3.1)

project(RecastNavigation)

# lib versions
SET(SOVERSION 1)
set(LIB_VERSION 1.6.0)
string(REPLACE "." "," LIB_VERSION_NUM "${LIB_VERSION}.0")

set_property(GLOBAL PROPERTY CXX_STANDARD 98)

option(RECASTNAVIGATION_DT_POLYREF64 "Use 64bit polyrefs instead of 32bit for Detour" ON)
option(RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER "Use dynamic dispatch for dtQueryFilter in Detour to allow for custom filters" ON)

if(MSVC AND BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

configure_file(
    "${RecastNavigation_SOURCE_DIR}/version.h.in"
    "${RecastNavigation_BINARY_DIR}/version.h")
install(FILES "${RecastNavigation_BINARY_DIR}/version.h" DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)

# Needed for recastnavigation.pc.in
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
set(libdir "${CMAKE_INSTALL_FULL_LIBDIR}")
set(bindir "${CMAKE_INSTALL_FULL_BINDIR}")
set(includedir "${CMAKE_INSTALL_FULL_INCLUDEDIR}")
set(PACKAGE_VERSION "${LIB_VERSION}")


file(GLOB SOURCES
	Recast/Source/*.cpp
	Detour/Source/*.cpp
	DetourCrowd/Source/*.cpp
	DetourTileCache/Source/*.cpp
	DebugUtils/Source/*.cpp
)

set(Recast_INCLUDE_DIR "${RecastNavigation_SOURCE_DIR}/Recast/Include")
set(Detour_INCLUDE_DIR "${RecastNavigation_SOURCE_DIR}/Detour/Include")
set(DetourCrowd_INCLUDE_DIR "${RecastNavigation_SOURCE_DIR}/DetourCrowd/Include")
set(DetourTileCache_INCLUDE_DIR "${RecastNavigation_SOURCE_DIR}/DetourTileCache/Include")
set(DebugUtils_INCLUDE_DIR "${RecastNavigation_SOURCE_DIR}/DebugUtils/Include")

file(GLOB INCLUDES
	${Recast_INCLUDE_DIR}/*.h
	${Detour_INCLUDE_DIR}/*.h
	${DetourCrowd_INCLUDE_DIR}/*.h
	${DetourTileCache_INCLUDE_DIR}/*.h
	${DebugUtils_INCLUDE_DIR}/*.h
)

set(TARGET_NAME recastnavigation)
add_library(${TARGET_NAME} ${SOURCES} ${INCLUDES})

if(RECASTNAVIGATION_DT_POLYREF64)
    target_compile_definitions(${TARGET_NAME} PUBLIC DT_POLYREF64=1)
endif()
if(RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER)
	target_compile_definitions(${TARGET_NAME} PUBLIC DT_VIRTUAL_QUERYFILTER=1)
endif()
target_compile_definitions(${TARGET_NAME} PUBLIC _ITERATOR_DEBUG_LEVEL=0)


target_include_directories(${TARGET_NAME} PUBLIC
    "$<BUILD_INTERFACE:${Recast_INCLUDE_DIR}>"
	"$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
	"$<BUILD_INTERFACE:${DetourCrowd_INCLUDE_DIR}>"
	"$<BUILD_INTERFACE:${DetourTileCache_INCLUDE_DIR}>"
	"$<BUILD_INTERFACE:${DebugUtils_INCLUDE_DIR}>"
)

if(MSVC)
   target_compile_options(${TARGET_NAME} PRIVATE /Zi)

   # Tell linker to include symbol data
    set_target_properties(${TARGET_NAME} PROPERTIES 
        LINK_FLAGS "/INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF"
    )

    # Set file name & location
    set_target_properties(${TARGET_NAME} PROPERTIES 
        COMPILE_PDB_NAME ${TARGET_NAME} 
        COMPILE_PDB_OUTPUT_DIR ${CMAKE_BINARY_DIR}
    )
endif()

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/recastnavigation-config.cmake.in
    recastnavigation-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/recastnavigation
)

write_basic_package_version_file(
    recastnavigation-config-version.cmake
    VERSION ${LIB_VERSION}
    COMPATIBILITY AnyNewerVersion
)

