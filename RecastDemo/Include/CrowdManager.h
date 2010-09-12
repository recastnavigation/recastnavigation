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

#ifndef CROWDMANAGER_H
#define CROWDMANAGER_H

#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "ValueHistory.h"


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
	
	float cornerVerts[AGENT_MAX_CORNERS*3];
	unsigned char cornerFlags[AGENT_MAX_CORNERS];
	dtPolyRef cornerPolys[AGENT_MAX_CORNERS];
	int ncorners;
	
	unsigned char active;
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
	dtObstacleAvoidanceQuery* m_obstacleQuery;
	
	int m_totalTime;
	int m_rvoTime;
	int m_sampleCount;
	
public:
	CrowdManager();
	~CrowdManager();
	
	void reset();
	const Agent* getAgent(const int idx);
	const int getAgentCount() const;
	int addAgent(const float* pos, const float radius, const float height);
	void removeAgent(const int idx);
	void setMoveTarget(const int idx, const float* pos);
	
	int getActiveAgents(Agent** agents, const int maxAgents);
	
	void update(const float dt, unsigned int flags, dtNavMeshQuery* navquery);
	
	const dtObstacleAvoidanceDebugData* getVODebugData(const int idx) const { return m_vodebug[idx]; }	
	inline int getTotalTime() const { return m_totalTime; }
	inline int getRVOTime() const { return m_rvoTime; }
	inline int getSampleCount() const { return m_sampleCount; }
};


#endif // CROWDMANAGER_H