#include "catch2/catch_amalgamated.hpp"
#include "Recast.h"

using Catch::Approx;

TEST_CASE("rcCalcBounds - basic bounding box", "[recast][bounds]")
{
    float verts[] = {
        1.0f,  2.0f, 3.0f,
        0.0f,  2.0f, 5.0f,
        4.0f,  1.0f, 6.0f,
        2.0f,  8.0f, 1.0f
    };

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, 4, bmin, bmax);

    REQUIRE(bmin[0] == Approx(0.0f));
    REQUIRE(bmax[0] == Approx(4.0f));

    REQUIRE(bmin[1] == Approx(1.0f));
    REQUIRE(bmax[1] == Approx(8.0f));

    REQUIRE(bmin[2] == Approx(1.0f));
    REQUIRE(bmax[2] == Approx(6.0f));
}

TEST_CASE("rcCalcBounds - single vertex", "[recast][bounds]")
{
    float verts[] = { 3.0f, 7.0f, -2.0f };

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, 1, bmin, bmax);

    REQUIRE(bmin[0] == Approx(3.0f));
    REQUIRE(bmin[1] == Approx(7.0f));
    REQUIRE(bmin[2] == Approx(-2.0f));
    REQUIRE(bmax[0] == Approx(3.0f));
    REQUIRE(bmax[1] == Approx(7.0f));
    REQUIRE(bmax[2] == Approx(-2.0f));
}

TEST_CASE("rcCalcBounds - negative coordinates", "[recast][bounds]")
{
    float verts[] = {
        -5.0f, -3.0f, -1.0f,
        -1.0f, -8.0f, -4.0f,
        -2.0f, -1.0f, -9.0f
    };

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, 3, bmin, bmax);

    REQUIRE(bmin[0] == Approx(-5.0f));
    REQUIRE(bmax[0] == Approx(-1.0f));

    REQUIRE(bmin[1] == Approx(-8.0f));
    REQUIRE(bmax[1] == Approx(-1.0f));

    REQUIRE(bmin[2] == Approx(-9.0f));
    REQUIRE(bmax[2] == Approx(-1.0f));
}

TEST_CASE("rcCalcBounds - mixed positive and negative", "[recast][bounds]")
{
    float verts[] = {
        -10.0f,  5.0f, -3.0f,
         10.0f, -5.0f,  3.0f,
          0.0f,  0.0f,  0.0f
    };

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, 3, bmin, bmax);

    REQUIRE(bmin[0] == Approx(-10.0f));
    REQUIRE(bmax[0] == Approx(10.0f));

    REQUIRE(bmin[1] == Approx(-5.0f));
    REQUIRE(bmax[1] == Approx(5.0f));

    REQUIRE(bmin[2] == Approx(-3.0f));
    REQUIRE(bmax[2] == Approx(3.0f));
}

TEST_CASE("rcCalcBounds - all identical vertices", "[recast][bounds]")
{
    float verts[] = {
        2.0f, 2.0f, 2.0f,
        2.0f, 2.0f, 2.0f,
        2.0f, 2.0f, 2.0f
    };

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, 3, bmin, bmax);

    REQUIRE(bmin[0] == Approx(2.0f));
    REQUIRE(bmin[1] == Approx(2.0f));
    REQUIRE(bmin[2] == Approx(2.0f));
    REQUIRE(bmax[0] == Approx(2.0f));
    REQUIRE(bmax[1] == Approx(2.0f));
    REQUIRE(bmax[2] == Approx(2.0f));
}

TEST_CASE("rcCalcBounds - large vertex count", "[recast][bounds]")
{
    const int nv = 300;
    float verts[nv * 3];

    for (int i = 0; i < nv; ++i)
    {
        verts[i * 3 + 0] = static_cast<float>(i);
        verts[i * 3 + 1] = static_cast<float>(i) * 0.5f;
        verts[i * 3 + 2] = static_cast<float>(nv - i);
    }

    float bmin[3] = {};
    float bmax[3] = {};

    rcCalcBounds(verts, nv, bmin, bmax);

    REQUIRE(bmin[0] == Approx(0.0f));
    REQUIRE(bmax[0] == Approx(299.0f));

    REQUIRE(bmin[1] == Approx(0.0f));
    REQUIRE(bmax[1] == Approx(149.5f));

    REQUIRE(bmin[2] == Approx(1.0f));
    REQUIRE(bmax[2] == Approx(300.0f));
}