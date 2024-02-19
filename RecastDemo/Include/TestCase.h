//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
#pragma once

#include <DetourNavMesh.h>
#include <string>

class dtNavMeshQuery;
class dtNavMesh;

class TestCase {
  enum TestType {
    TEST_PATHFIND,
    TEST_RAYCAST
  };

  struct Test {
    Test() : type(),
             spos(),
             epos(),
             nspos(),
             nepos(),
             radius(0),
             includeFlags(0),
             excludeFlags(0),
             expand(false),
             straight(nullptr),
             nstraight(0),
             polys(nullptr),
             npolys(0),
             findNearestPolyTime(0),
             findPathTime(0),
             findStraightPathTime(0),
             next(nullptr) {
    }

    ~Test() {
      delete[] straight;
      delete[] polys;
    }
    Test(const Test &) = delete;
    Test &operator=(const Test &) = delete;

    TestType type;
    float spos[3];
    float epos[3];
    float nspos[3];
    float nepos[3];
    float radius;
    unsigned short includeFlags;
    unsigned short excludeFlags;
    bool expand;

    float *straight;
    int nstraight;
    dtPolyRef *polys;
    int npolys;

    int findNearestPolyTime;
    int findPathTime;
    int findStraightPathTime;

    Test *next;
  };

  std::string m_sampleName;
  std::string m_geomFileName;
  Test *m_tests;

  void resetTimes() const;

public:
  TestCase();
  ~TestCase();

  bool load(const std::string &filePath);

  const std::string &getSampleName() const { return m_sampleName; }
  const std::string &getGeomFileName() const { return m_geomFileName; }

  void doTests(const dtNavMesh *navmesh, const dtNavMeshQuery *navquery) const;

  void handleRender() const;
  bool handleRenderOverlay(const double *proj, const double *model, const int *view) const;

private:
  // Explicitly disabled copy constructor and copy assignment operator.
  TestCase(const TestCase &);
  TestCase &operator=(const TestCase &);
};
