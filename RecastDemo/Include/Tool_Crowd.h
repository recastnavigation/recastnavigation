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
#include "Sample.h"
#include "ValueHistory.h"

struct CrowdToolParams
{
	bool showCorners = false;
	bool showCollisionSegments = false;
	bool showPath = false;
	bool showVO = false;
	bool showOpt = false;
	bool showNeighbors = false;

	bool showLabels = false;
	bool showGrid = false;
	bool showNodes = false;
	bool showPerfGraph = false;
	bool showDetailAll = false;

	bool anticipateTurns = true;
	bool optimizeVis = true;
	bool optimizeTopo = true;
	bool obstacleAvoidance = true;
	int obstacleAvoidanceType = 1;
	bool separation = false;
	float separationWeight = 2.0f;
};

/// Tool to create crowds.
class CrowdToolState : public SampleToolState
{
	Sample* sample = nullptr;

	float targetPosition[3]{};
	dtPolyRef targetPolyRef{0};

	dtCrowdAgentDebugInfo agentDebug;
	dtObstacleAvoidanceDebugData* obstacleAvoidanceDebugData = nullptr;

	static constexpr int AGENT_MAX_TRAIL = 64;
	static constexpr int MAX_AGENTS = 128;

	struct AgentTrail
	{
		float trail[AGENT_MAX_TRAIL * 3];
		int htrail;
	};
	AgentTrail trails[MAX_AGENTS];

	ValueHistory crowdTotalTime;
	ValueHistory crowdSampleCount;

	CrowdToolParams toolParams;

	bool run = true;

public:
	CrowdToolState();
	CrowdToolState(const CrowdToolState&) = delete;
	CrowdToolState(const CrowdToolState&&) = delete;
	CrowdToolState& operator=(const CrowdToolState&) = delete;
	CrowdToolState& operator=(const CrowdToolState&&) = delete;
	~CrowdToolState() override;

	void init(Sample* newSample) override;
	void reset() override;
	void render() override;
	void renderOverlay(double* proj, double* model, int* view) override;
	void handleUpdate(float dt) override;

	[[nodiscard]] bool isRunning() const { return run; }
	void setRunning(const bool s) { run = s; }

	void addAgent(const float* pos);
	void removeAgent(int idx);
	void highlightAgent(int idx);
	void updateAgentParams();
	int hitTestAgents(const float* s, const float* p);
	void setMoveTarget(const float* p, bool adjust);
	void updateTick(float dt);

	CrowdToolParams* getToolParams() { return &toolParams; }
};

class CrowdTool : public SampleTool
{
	Sample* sample = nullptr;
	CrowdToolState* state = nullptr;

	enum class ToolMode : uint8_t
	{
		CREATE,
		MOVE_TARGET,
		SELECT,
		TOGGLE_POLYS
	};
	ToolMode mode = ToolMode::CREATE;

public:
	SampleToolType type() override { return SampleToolType::CROWD; }
	void init(Sample* sample) override;
	void reset() override;
	void drawMenuUI() override;
	void onClick(const float* s, const float* p, bool shift) override;
	void onToggle() override;
	void singleStep() override;
	void update(float dt) override;
	void render() override;
	void drawOverlayUI(double* proj, double* model, int* view) override;
};
