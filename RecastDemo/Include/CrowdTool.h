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

#include <cstdint>

struct CrowdToolParams
{
	bool m_expandSelectedDebugDraw = true;
	bool m_showCorners = false;
	bool m_showCollisionSegments = false;
	bool m_showPath = false;
	bool m_showVO = false;
	bool m_showOpt = false;
	bool m_showNeis = false;

	bool m_expandDebugDraw = false;
	bool m_showLabels = false;
	bool m_showGrid = false;
	bool m_showNodes = false;
	bool m_showPerfGraph = false;
	bool m_showDetailAll = false;

	bool m_expandOptions = true;
	bool m_anticipateTurns = true;
	bool m_optimizeVis = true;
	bool m_optimizeTopo = true;
	bool m_obstacleAvoidance = true;
	float m_obstacleAvoidanceType = 3.0f;
	bool m_separation = false;
	float m_separationWeight = 2.0f;
};

/// Tool to create crowds.
class CrowdToolState : public SampleToolState
{
	Sample* m_sample;
	dtNavMesh* m_nav;
	dtCrowd* m_crowd;

	float m_targetPos[3];
	dtPolyRef m_targetRef;

	dtCrowdAgentDebugInfo m_agentDebug;
	dtObstacleAvoidanceDebugData* m_vod;

	static const int AGENT_MAX_TRAIL = 64;
	static const int MAX_AGENTS = 128;
	struct AgentTrail
	{
		float trail[AGENT_MAX_TRAIL * 3];
		int htrail;
	};
	AgentTrail m_trails[MAX_AGENTS];

	ValueHistory m_crowdTotalTime;
	ValueHistory m_crowdSampleCount;

	CrowdToolParams m_toolParams;

	bool m_run;

public:
	CrowdToolState();
	CrowdToolState(const CrowdToolState&) = delete;
	CrowdToolState(const CrowdToolState&&) = delete;
	CrowdToolState& operator=(const CrowdToolState&) = delete;
	CrowdToolState& operator=(const CrowdToolState&&) = delete;
	~CrowdToolState() override;

	void init(class Sample* sample) override;
	void reset() override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
	void handleUpdate(const float dt) override;

	inline bool isRunning() const { return m_run; }
	inline void setRunning(const bool s) { m_run = s; }

	void addAgent(const float* pos);
	void removeAgent(const int idx);
	void hilightAgent(const int idx);
	void updateAgentParams();
	int hitTestAgents(const float* s, const float* p);
	void setMoveTarget(const float* p, bool adjust);
	void updateTick(const float dt);

	inline CrowdToolParams* getToolParams() { return &m_toolParams; }
};

class CrowdTool : public SampleTool
{
	Sample* m_sample = nullptr;
	CrowdToolState* m_state = nullptr;

	enum class ToolMode : uint8_t
	{
		CREATE,
		MOVE_TARGET,
		SELECT,
		TOGGLE_POLYS
	};
	ToolMode m_mode = ToolMode::CREATE;

public:
	SampleToolType type() override { return SampleToolType::CROWD; }
	void init(Sample* sample) override;
	void reset() override;
	void handleMenu() override;
	void handleClick(const float* s, const float* p, bool shift) override;
	void handleToggle() override;
	void handleStep() override;
	void handleUpdate(const float dt) override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
};
