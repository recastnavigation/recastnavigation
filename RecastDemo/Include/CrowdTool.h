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
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourObstacleAvoidance.h"
#include "Sample.h"
#include "ValueHistory.h"

// Tool to create crowds.

struct CrowdToolParams {
  bool m_expandSelectedDebugDraw;
  bool m_showCorners;
  bool m_showCollisionSegments;
  bool m_showPath;
  bool m_showVO;
  bool m_showOpt;
  bool m_showNeis;

  bool m_expandDebugDraw;
  bool m_showLabels;
  bool m_showGrid;
  bool m_showNodes;
  bool m_showPerfGraph;
  bool m_showDetailAll;

  bool m_expandOptions;
  bool m_anticipateTurns;
  bool m_optimizeVis;
  bool m_optimizeTopo;
  bool m_obstacleAvoidance;
  float m_obstacleAvoidanceType;
  bool m_separation;
  float m_separationWeight;
};

class CrowdToolState final : public SampleToolState {
  Sample *m_sample{};
  dtNavMesh *m_nav{};
  dtCrowd *m_crowd{};

  float m_targetPos[3]{};
  dtPolyRef m_targetRef{};

  dtCrowdAgentDebugInfo m_agentDebug{};
  dtObstacleAvoidanceDebugData *m_vod{};

  static constexpr int AGENT_MAX_TRAIL = 64;
  static constexpr int MAX_AGENTS = 128;
  struct AgentTrail {
    float trail[AGENT_MAX_TRAIL * 3]{};
    int htrail{};
  };
  AgentTrail m_trails[MAX_AGENTS]{};

  ValueHistory m_crowdTotalTime{};
  ValueHistory m_crowdSampleCount{};

  CrowdToolParams m_toolParams{};

  bool m_run{true};

public:
  CrowdToolState();
  ~CrowdToolState() override;

  void init(Sample *sample) override;
  void reset() override;
  void handleRender() override;
  void handleRenderOverlay(double *proj, double *model, int *view) override;
  void handleUpdate(float dt) override;

  bool isRunning() const { return m_run; }
  void setRunning(const bool s) { m_run = s; }

  void addAgent(const float *pos);
  void removeAgent(int idx);
  void hilightAgent(int idx);
  void updateAgentParams() const;
  int hitTestAgents(const float *s, const float *p) const;
  void setMoveTarget(const float *p, bool adjust);
  void updateTick(float dt);

  CrowdToolParams *getToolParams() { return &m_toolParams; }

  // Explicitly disabled copy constructor and copy assignment operator.
  CrowdToolState(const CrowdToolState &) = delete;
  CrowdToolState &operator=(const CrowdToolState &) = delete;
};

class CrowdTool final : public SampleTool {
  Sample *m_sample;
  CrowdToolState *m_state;

  enum ToolMode {
    TOOLMODE_CREATE,
    TOOLMODE_MOVE_TARGET,
    TOOLMODE_SELECT,
    TOOLMODE_TOGGLE_POLYS
  };
  ToolMode m_mode;

public:
  CrowdTool();

  int type() override { return TOOL_CROWD; }
  void init(Sample *sample) override;
  void reset() override;
  void handleMenu() override;
  void handleClick(const float *s, const float *p, bool shift) override;
  void handleToggle() override;
  void handleStep() override;
  void handleUpdate(float dt) override;
  void handleRender() override;
  void handleRenderOverlay(double *proj, double *model, int *view) override;
};
