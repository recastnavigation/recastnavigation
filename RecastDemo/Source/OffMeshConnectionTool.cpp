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

#include <SDL.h>
#include <SDL_opengl.h>
#include <cfloat>
#include <cmath>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <gl/GLU.h>
#endif

#include "DetourDebugDraw.h"
#include "InputGeom.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "Sample.h"
#include "imgui.h"

OffMeshConnectionTool::OffMeshConnectionTool() : m_sample(nullptr), m_hitPos{},
                                                 m_hitPosSet(false),
                                                 m_bidir(true),
                                                 m_oldFlags(0) {
}

OffMeshConnectionTool::~OffMeshConnectionTool() {
  if (m_sample) {
    m_sample->setNavMeshDrawFlags(m_oldFlags);
  }
}

void OffMeshConnectionTool::init(Sample *sample) {
  if (m_sample != sample) {
    m_sample = sample;
    m_oldFlags = m_sample->getNavMeshDrawFlags();
    m_sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_OFFMESHCONS);
  }
}

void OffMeshConnectionTool::reset() {
  m_hitPosSet = false;
}

void OffMeshConnectionTool::handleMenu() {
  if (imguiCheck("One Way", !m_bidir))
    m_bidir = false;
  if (imguiCheck("Bidirectional", m_bidir))
    m_bidir = true;
}

void OffMeshConnectionTool::handleClick(const float * /*s*/, const float *p, const bool shift) {
  if (!m_sample)
    return;
  InputGeom *geom = m_sample->getInputGeom();
  if (!geom)
    return;

  if (shift) {
    // Delete
    // Find nearest link end-point
    float nearestDist = FLT_MAX;
    int nearestIndex = -1;
    const float *verts = geom->getOffMeshConnectionVerts();
    for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i) {
      const float *v = &verts[i * 3];
      const float d = rcVdistSqr(p, v);
      if (d < nearestDist) {
        nearestDist = d;
        nearestIndex = i / 2; // Each link has two vertices.
      }
    }
    // If end point close enough, delete it.
    if (nearestIndex != -1 &&
        std::sqrt(nearestDist) < m_sample->getAgentRadius()) {
      geom->deleteOffMeshConnection(nearestIndex);
    }
  } else {
    // Create
    if (!m_hitPosSet) {
      rcVcopy(m_hitPos, p);
      m_hitPosSet = true;
    } else {
      constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
      constexpr uint16_t flags = SAMPLE_POLYFLAGS_JUMP;
      geom->addOffMeshConnection(m_hitPos, p, m_sample->getAgentRadius(), m_bidir ? 1 : 0, area, flags);
      m_hitPosSet = false;
    }
  }
}

void OffMeshConnectionTool::handleToggle() {
}

void OffMeshConnectionTool::handleStep() {
}

void OffMeshConnectionTool::handleUpdate(const float /*dt*/) {
}

void OffMeshConnectionTool::handleRender() {
  duDebugDraw &dd = m_sample->getDebugDraw();
  const float s = m_sample->getAgentRadius();

  if (m_hitPosSet)
    duDebugDrawCross(&dd, m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2], s, duRGBA(0, 0, 0, 128), 2.0f);

  const InputGeom *geom = m_sample->getInputGeom();
  if (geom)
    geom->drawOffMeshConnections(&dd, true);
}

void OffMeshConnectionTool::handleRenderOverlay(double *proj, double *model, int *view) {
  GLdouble x, y, z;

  // Draw start and end point labels
  if (m_hitPosSet && gluProject(m_hitPos[0], m_hitPos[1], m_hitPos[2],
                                model, proj, view, &x, &y, &z)) {
    imguiDrawText(static_cast<int>(x), static_cast<int>(y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
  }

  // Tool help
  const int h = view[3];
  if (!m_hitPosSet) {
    imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Create new connection.  SHIFT+LMB: Delete existing connection, click close to start or end point.", imguiRGBA(255, 255, 255, 192));
  } else {
    imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Set connection end point and finish.", imguiRGBA(255, 255, 255, 192));
  }
}
