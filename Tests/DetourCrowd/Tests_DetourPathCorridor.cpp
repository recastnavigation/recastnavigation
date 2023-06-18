#include "catch2/catch_all.hpp"

#include "DetourPathCorridor.h"

TEST_CASE("dtMergeCorridorStartMoved")
{
    SECTION("Should handle empty input")
    {
        dtPolyRef* const path = nullptr;
        const int npath = 0;
        const int maxPath = 0;
        const dtPolyRef* const visited = nullptr;
        const int nvisited = 0;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 0);
    }

    SECTION("Should handle empty visited")
    {
        dtPolyRef path[] = {1};
        const int npath = 1;
        const int maxPath = 1;
        const dtPolyRef* const visited = nullptr;
        const int nvisited = 0;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 1);
        const dtPolyRef expectedPath[] = {1};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should handle empty path")
    {
        dtPolyRef* const path = nullptr;
        const int npath = 0;
        const int maxPath = 0;
        const dtPolyRef visited[] = {1};
        const int nvisited = 1;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 0);
    }

    SECTION("Should strip visited points from path except last")
    {
        dtPolyRef path[] = {1, 2};
        const int npath = 2;
        const int maxPath = 2;
        const dtPolyRef visited[] = {1, 2};
        const int nvisited = 2;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 1);
        const dtPolyRef expectedPath[] = {2, 2};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should add visited points not present in path in reverse order")
    {
        dtPolyRef path[] = {1, 2, 0};
        const int npath = 2;
        const int maxPath = 3;
        const dtPolyRef visited[] = {1, 2, 3, 4};
        const int nvisited = 4;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 3);
        const dtPolyRef expectedPath[] = {4, 3, 2};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should add visited points not present in path up to the path capacity")
    {
        dtPolyRef path[] = {1, 2, 0};
        const int npath = 2;
        const int maxPath = 3;
        const dtPolyRef visited[] = {1, 2, 3, 4, 5};
        const int nvisited = 5;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 3);
        const dtPolyRef expectedPath[] = {5, 4, 3};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should not change path if there is no intersection with visited")
    {
        dtPolyRef path[] = {1, 2};
        const int npath = 2;
        const int maxPath = 2;
        const dtPolyRef visited[] = {3, 4};
        const int nvisited = 2;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 2);
        const dtPolyRef expectedPath[] = {1, 2};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should save unvisited path points")
    {
        dtPolyRef path[] = {1, 2, 0};
        const int npath = 2;
        const int maxPath = 3;
        const dtPolyRef visited[] = {1, 3};
        const int nvisited = 2;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 3);
        const dtPolyRef expectedPath[] = {3, 1, 2};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }

    SECTION("Should save unvisited path points up to the path capacity")
    {
        dtPolyRef path[] = {1, 2};
        const int npath = 2;
        const int maxPath = 2;
        const dtPolyRef visited[] = {1, 3};
        const int nvisited = 2;
        const int result = dtMergeCorridorStartMoved(path, npath, maxPath, visited, nvisited);
        CHECK(result == 2);
        const dtPolyRef expectedPath[] = {3, 1};
        CHECK_THAT(path, Catch::Matchers::RangeEquals(expectedPath));
    }
}
