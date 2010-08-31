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

#ifndef CROWDTOOL_H
#define CROWDTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourObstacleAvoidance.h"
#include "ValueHistory.h"

// Tool to create crowds.

enum AgentTargetState
{
	AGENT_TARGET_NONE = 0,
	AGENT_TARGET_SET = 1,
	AGENT_TARGET_ACQUIRED = 2,
	AGENT_TARGET_PATH = 3,
	AGENT_TARGET_FAILED = 4,
};

static const int AGENT_MAX_PATH = 256;
static const int AGENT_MAX_CORNERS = 4;
static const int AGENT_MAX_TRAIL = 64;
static const int AGENT_MAX_COLSEGS = 32;
static const int AGENT_MAX_NEIS = 8;

enum AgentApproach
{
	AGENT_APPROACH_CORNER = 0,
	AGENT_APPROACH_OFFMESH_CON = 0,
	AGENT_APPROACH_END = 0,
};

struct Agent
{
	float pos[3];
	float radius, height;

	float dvel[3];
	float nvel[3];
	float vel[3];
	float npos[3];
	float disp[3];

	float opts[3], opte[3];

	float maxspeed;
	float t;
	float var;

	float colradius;
	float colcenter[3];
	float colsegs[AGENT_MAX_COLSEGS*6];
	int ncolsegs;

	float trail[AGENT_MAX_TRAIL*3];
	int htrail;

	unsigned char targetState;
	float target[3];
	dtPolyRef targetRef;

	dtPolyRef path[AGENT_MAX_PATH];
	int npath;
	float corners[AGENT_MAX_CORNERS*3];
	int ncorners;

	unsigned char active;
};



struct Isect
{
	float u;
	int inside;
};

static const int FORM_MAX_ISECT = 32;
static const int FORM_MAX_SEGS = 16;
static const int FORM_MAX_POLYS = 32;

struct FormationSeg
{
	float p[3], q[3];
	Isect ints[FORM_MAX_ISECT];
	int nints;
};

struct Formation
{
	FormationSeg segs[FORM_MAX_SEGS];
	int nsegs;
	dtPolyRef polys[FORM_MAX_POLYS];
	int npolys;
};


enum UpdateFlags
{
	CROWDMAN_ANTICIPATE_TURNS = 1,
	CROWDMAN_USE_VO = 2,
	CROWDMAN_DRUNK = 4,
};

class CrowdManager
{
	static const int MAX_AGENTS = 32;
	Agent m_agents[MAX_AGENTS];
	dtObstacleAvoidanceDebugData* m_vodebug[MAX_AGENTS];

	ValueHistory m_totalTime;
	ValueHistory m_rvoTime;
	ValueHistory m_sampleCount;
	
	dtObstacleAvoidanceQuery* m_obstacleQuery;

public:
	CrowdManager();
	~CrowdManager();
	
	void reset();
	const Agent* getAgent(const int idx);
	const int getAgentCount() const;
	int addAgent(const float* pos, const float radius, const float height);
	void removeAgent(const int idx);
	void setMoveTarget(const int idx, const float* pos);

	void update(const float dt, unsigned int flags, dtNavMeshQuery* navquery);

	const dtObstacleAvoidanceDebugData* getVODebugData(const int idx) const { return m_vodebug[idx]; }

	const ValueHistory* getTotalTimeGraph() const { return &m_totalTime; }
	const ValueHistory* getRVOTimeGraph() const { return &m_rvoTime; }
	const ValueHistory* getSampleCountGraph() const { return &m_sampleCount; }
};

class CrowdTool : public SampleTool
{
	Sample* m_sample;
	float m_targetPos[3];
	bool m_targetPosSet;
	
	Formation m_form;
	
	bool m_expandDebugDraw;
	bool m_showLabels;
	bool m_showCorners;
	bool m_showTargets;
	bool m_showCollisionSegments;
	bool m_showPath;
	bool m_showVO;
	bool m_showOpt;
	
	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_useVO;
	bool m_drunkMove;
	
	bool m_run;
	
	CrowdManager m_crowd;
		
	enum ToolMode
	{
		TOOLMODE_CREATE,
		TOOLMODE_MOVE,
	};
	ToolMode m_mode;
	
public:
	CrowdTool();
	~CrowdTool();
	
	virtual int type() { return TOOL_CROWD; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
};

#endif // CROWDTOOL_H
