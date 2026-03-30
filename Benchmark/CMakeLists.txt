
add_executable(rcCalcBoundsBench
    rcCalcBounds_bench.cpp
)

set_target_properties(rcCalcBoundsBench PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(rcCalcBoundsBench PRIVATE
    ${CMAKE_SOURCE_DIR}/Recast/Include
)

target_link_libraries(rcCalcBoundsBench PRIVATE
    Recast
)