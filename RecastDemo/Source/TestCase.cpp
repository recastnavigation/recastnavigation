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

#include "TestCase.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <new>
#include <sstream>

#include <SDL.h>
#include <SDL_opengl.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <gl/GLU.h>
#endif

#include "PerfTimer.h"
#include "imgui.h"

TestCase::TestCase() : m_tests(nullptr) {
}

TestCase::~TestCase() {
  const Test *iter = m_tests;
  while (iter) {
    const Test *next = iter->next;
    delete iter;
    iter = next;
  }
}

static char *parseRow(char *buf, const char *bufEnd, char *row, const int len) {
  bool start = true;
  bool done = false;
  int n = 0;
  while (!done && buf < bufEnd) {
    const char c = *buf;
    buf++;
    // multirow
    switch (c) {
    case '\n':
      if (start)
        break;
      done = true;
      break;
    case '\r':
      break;
    case '\t':
    case ' ':
      if (start)
        break;
    // else falls through
    default:
      start = false;
      row[n++] = c;
      if (n >= len - 1)
        done = true;
      break;
    }
  }
  row[n] = '\0';
  return buf;
}

static void copyName(std::string &dst, const char *src) {
  // Skip white spaces
  while (*src && isspace(*src))
    src++;
  dst = src;
}

bool TestCase::load(const std::string &filePath) {
  std::ifstream file(filePath, std::ios::in | std::ios::binary | std::ios::ate);

  if (!file.is_open()) {
    return false;
  }

  const std::streamsize fileSize = file.tellg();
  file.seekg(0, std::ios::beg);

  if (fileSize < 0) {
    return false;
  }

  const auto buf = new (std::nothrow) char[fileSize];
  if (!buf) {
    file.close();
    return false;
  }
  file.read(buf, fileSize);
  const std::size_t readLen = file.gcount();
  file.close();
  if (readLen != fileSize) {
    delete[] buf;
    return false;
  }

  char *src = buf;
  const char *srcEnd = buf + fileSize;
  char row[512];
  while (src < srcEnd) {
    // Parse one row
    row[0] = '\0';
    src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
    if (row[0] == 's') {
      // Sample name.
      copyName(m_sampleName, row + 1);
    } else if (row[0] == 'f') {
      // File name.
      copyName(m_geomFileName, row + 1);
    } else if (row[0] == 'p' && row[1] == 'f') {
      // Pathfind test.
      const auto test = new Test();
      test->type = TEST_PATHFIND;
      test->expand = false;
      test->next = m_tests;
      m_tests = test;
      std::istringstream rowStream(row + 2);
      rowStream >> test->spos[0] >> test->spos[1] >> test->spos[2] >> test->epos[0] >> test->epos[1] >> test->epos[2] >> std::hex >> test->includeFlags >> test->excludeFlags;
    } else if (row[0] == 'r' && row[1] == 'c') {
      // Pathfind test.
      const auto test = new Test();
      test->type = TEST_RAYCAST;
      test->expand = false;
      test->next = m_tests;
      m_tests = test;

      std::istringstream rowStream(row + 2);
      rowStream >> test->spos[0] >> test->spos[1] >> test->spos[2] >> test->epos[0] >> test->epos[1] >> test->epos[2] >> std::hex >> test->includeFlags >> test->excludeFlags;
    }
  }

  delete[] buf;

  return true;
}

void TestCase::resetTimes() const {
  for (Test *iter = m_tests; iter; iter = iter->next) {
    iter->findNearestPolyTime = 0;
    iter->findPathTime = 0;
    iter->findStraightPathTime = 0;
  }
}

void TestCase::doTests(const dtNavMesh *navmesh, const dtNavMeshQuery *navquery) const {
  if (!navmesh || !navquery)
    return;

  resetTimes();

  static constexpr int MAX_POLYS = 256;
  constexpr float polyPickExt[3] = {2, 4, 2};

  for (Test *iter = m_tests; iter; iter = iter->next) {
    dtPolyRef polys[MAX_POLYS];
    delete[] iter->polys;
    iter->polys = nullptr;
    iter->npolys = 0;
    delete[] iter->straight;
    iter->straight = nullptr;
    iter->nstraight = 0;

    dtQueryFilter filter;
    filter.setIncludeFlags(iter->includeFlags);
    filter.setExcludeFlags(iter->excludeFlags);

    // Find start points
    const TimeVal findNearestPolyStart = getPerfTime();

    dtPolyRef startRef, endRef;
    navquery->findNearestPoly(iter->spos, polyPickExt, &filter, &startRef, iter->nspos);
    navquery->findNearestPoly(iter->epos, polyPickExt, &filter, &endRef, iter->nepos);

    const TimeVal findNearestPolyEnd = getPerfTime();
    iter->findNearestPolyTime += getPerfTimeUsec(findNearestPolyEnd - findNearestPolyStart);

    if (!startRef || !endRef)
      continue;

    if (iter->type == TEST_PATHFIND) {
      float straight[MAX_POLYS * 3];
      // Find path
      const TimeVal findPathStart = getPerfTime();

      navquery->findPath(startRef, endRef, iter->spos, iter->epos, &filter, polys, &iter->npolys, MAX_POLYS);

      const TimeVal findPathEnd = getPerfTime();
      iter->findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

      // Find straight path
      if (iter->npolys) {
        const TimeVal findStraightPathStart = getPerfTime();

        navquery->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
                                   straight, nullptr, nullptr, &iter->nstraight, MAX_POLYS);
        const TimeVal findStraightPathEnd = getPerfTime();
        iter->findStraightPathTime += getPerfTimeUsec(findStraightPathEnd - findStraightPathStart);
      }

      // Copy results
      if (iter->npolys) {
        iter->polys = new dtPolyRef[iter->npolys];
        std::memcpy(iter->polys, polys, sizeof(dtPolyRef) * iter->npolys);
      }
      if (iter->nstraight) {
        iter->straight = new float[iter->nstraight * 3];
        std::memcpy(iter->straight, straight, sizeof(float) * 3 * iter->nstraight);
      }
    } else if (iter->type == TEST_RAYCAST) {
      float t = 0;
      float hitNormal[3], hitPos[3];

      iter->straight = new float[2 * 3];
      iter->nstraight = 2;

      iter->straight[0] = iter->spos[0];
      iter->straight[1] = iter->spos[1];
      iter->straight[2] = iter->spos[2];

      const TimeVal findPathStart = getPerfTime();

      navquery->raycast(startRef, iter->spos, iter->epos, &filter, &t, hitNormal, polys, &iter->npolys,
                        MAX_POLYS);

      const TimeVal findPathEnd = getPerfTime();
      iter->findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

      if (t > 1) {
        // No hit
        dtVcopy(hitPos, iter->epos);
      } else {
        // Hit
        dtVlerp(hitPos, iter->spos, iter->epos, t);
      }
      // Adjust height.
      if (iter->npolys > 0) {
        float h = 0;
        navquery->getPolyHeight(polys[iter->npolys - 1], hitPos, &h);
        hitPos[1] = h;
      }
      dtVcopy(&iter->straight[3], hitPos);

      if (iter->npolys) {
        iter->polys = new dtPolyRef[iter->npolys];
        std::memcpy(iter->polys, polys, sizeof(dtPolyRef) * iter->npolys);
      }
    }
  }

  std::cout << "Test Results:\n";
  int n = 0;
  for (const Test *iter = m_tests; iter; iter = iter->next) {
    const int total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
    std::cout << " - Path %02d:   " << static_cast<float>(total) * 1e-3f << " ms\n";
    std::cout << "    - poly:     " << static_cast<float>(iter->findNearestPolyTime) * 1e-3f << " ms\n";
    std::cout << "    - path:     " << static_cast<float>(iter->findPathTime) * 1e-3f << " ms\n";
    std::cout << "    - straight: " << static_cast<float>(iter->findStraightPathTime) * 1e-3f << " ms\n";
    n++;
  }
  std::cout << std::flush;
}

void TestCase::handleRender() const {
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  for (const Test *iter = m_tests; iter; iter = iter->next) {
    float dir[3];
    dtVsub(dir, iter->epos, iter->spos);
    dtVnormalize(dir);
    glColor4ub(128, 25, 0, 192);
    glVertex3f(iter->spos[0], iter->spos[1] - 0.3f, iter->spos[2]);
    glVertex3f(iter->spos[0], iter->spos[1] + 0.3f, iter->spos[2]);
    glVertex3f(iter->spos[0], iter->spos[1] + 0.3f, iter->spos[2]);
    glVertex3f(iter->spos[0] + dir[0] * 0.3f, iter->spos[1] + 0.3f + dir[1] * 0.3f, iter->spos[2] + dir[2] * 0.3f);
    glColor4ub(51, 102, 0, 129);
    glVertex3f(iter->epos[0], iter->epos[1] - 0.3f, iter->epos[2]);
    glVertex3f(iter->epos[0], iter->epos[1] + 0.3f, iter->epos[2]);

    if (iter->expand) {
      constexpr float s = 0.1f;
      glColor4ub(255, 32, 0, 128);
      glVertex3f(iter->spos[0] - s, iter->spos[1], iter->spos[2]);
      glVertex3f(iter->spos[0] + s, iter->spos[1], iter->spos[2]);
      glVertex3f(iter->spos[0], iter->spos[1], iter->spos[2] - s);
      glVertex3f(iter->spos[0], iter->spos[1], iter->spos[2] + s);
      glColor4ub(255, 192, 0, 255);
      glVertex3f(iter->nspos[0] - s, iter->nspos[1], iter->nspos[2]);
      glVertex3f(iter->nspos[0] + s, iter->nspos[1], iter->nspos[2]);
      glVertex3f(iter->nspos[0], iter->nspos[1], iter->nspos[2] - s);
      glVertex3f(iter->nspos[0], iter->nspos[1], iter->nspos[2] + s);

      glColor4ub(255, 32, 0, 128);
      glVertex3f(iter->epos[0] - s, iter->epos[1], iter->epos[2]);
      glVertex3f(iter->epos[0] + s, iter->epos[1], iter->epos[2]);
      glVertex3f(iter->epos[0], iter->epos[1], iter->epos[2] - s);
      glVertex3f(iter->epos[0], iter->epos[1], iter->epos[2] + s);
      glColor4ub(255, 192, 0, 255);
      glVertex3f(iter->nepos[0] - s, iter->nepos[1], iter->nepos[2]);
      glVertex3f(iter->nepos[0] + s, iter->nepos[1], iter->nepos[2]);
      glVertex3f(iter->nepos[0], iter->nepos[1], iter->nepos[2] - s);
      glVertex3f(iter->nepos[0], iter->nepos[1], iter->nepos[2] + s);
    }

    if (iter->expand)
      glColor4ub(255, 192, 0, 255);
    else
      glColor4ub(0, 0, 0, 64);

    for (int i = 0; i < iter->nstraight - 1; ++i) {
      glVertex3f(iter->straight[i * 3 + 0], iter->straight[i * 3 + 1] + 0.3f, iter->straight[i * 3 + 2]);
      glVertex3f(iter->straight[(i + 1) * 3 + 0], iter->straight[(i + 1) * 3 + 1] + 0.3f,
                 iter->straight[(i + 1) * 3 + 2]);
    }
  }
  glEnd();
  glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(const double *proj, const double *model, const int *view) const {
  GLdouble x, y, z;
  char text[64];
  int n = 0;

  static constexpr float LABEL_DIST = 1.0f;

  for (const Test *iter = m_tests; iter; iter = iter->next) {
    float pt[3], dir[3];
    if (iter->nstraight) {
      dtVcopy(pt, &iter->straight[3]);
      if (dtVdist(pt, iter->spos) > LABEL_DIST) {
        dtVsub(dir, pt, iter->spos);
        dtVnormalize(dir);
        dtVmad(pt, iter->spos, dir, LABEL_DIST);
      }
      pt[1] += 0.5f;
    } else {
      dtVsub(dir, iter->epos, iter->spos);
      dtVnormalize(dir);
      dtVmad(pt, iter->spos, dir, LABEL_DIST);
      pt[1] += 0.5f;
    }

    if (gluProject(pt[0], pt[1], pt[2],
                   model, proj, view, &x, &y, &z)) {
      std::snprintf(text, sizeof(text), "Path %d\n", n);
      uint32_t col = imguiRGBA(0, 0, 0, 128);
      if (iter->expand)
        col = imguiRGBA(255, 192, 0, 220);
      imguiDrawText(static_cast<int>(x), static_cast<int>(y - 25), IMGUI_ALIGN_CENTER, text, col);
    }
    n++;
  }

  static int resScroll = 0;
  const bool mouseOverMenu = imguiBeginScrollArea("Test Results", 10, view[3] - 10 - 350, 200, 350, &resScroll);
  //		mouseOverMenu = true;

  n = 0;
  for (Test *iter = m_tests; iter; iter = iter->next) {
    char subtext[64];
    const int total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
    std::snprintf(subtext, sizeof(subtext), "%.4f ms", static_cast<float>(total) / 1000.0f);
    std::snprintf(text, sizeof(text), "Path %d", n);

    if (imguiCollapse(text, subtext, iter->expand))
      iter->expand = !iter->expand;
    if (iter->expand) {
      std::snprintf(text, sizeof(text), "Poly: %.4f ms", static_cast<float>(iter->findNearestPolyTime) / 1000.0f);
      imguiValue(text);

      std::snprintf(text, sizeof(text), "Path: %.4f ms", static_cast<float>(iter->findPathTime) / 1000.0f);
      imguiValue(text);

      std::snprintf(text, sizeof(text), "Straight: %.4f ms", static_cast<float>(iter->findStraightPathTime) / 1000.0f);
      imguiValue(text);

      imguiSeparator();
    }

    n++;
  }

  imguiEndScrollArea();

  return mouseOverMenu;
}
