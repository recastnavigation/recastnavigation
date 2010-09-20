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


/*enum AgentTargetState
{
	AGENT_TARGET_NONE = 0,
	AGENT_TARGET_SET = 1,
	AGENT_TARGET_ACQUIRED = 2,
	AGENT_TARGET_WAITING_FOR_PATH = 3,
	AGENT_TARGET_PATH = 4,
	AGENT_TARGET_FAILED = 5,
};*/

static const int AGENT_MAX_PATH = 256;
static const int AGENT_MAX_CORNERS = 4;
static const int AGENT_MAX_TRAIL = 64;
static const int AGENT_MAX_LOCALSEGS = 32;
static const int AGENT_MAX_NEIS = 8;

static const unsigned int PATHQ_INVALID = 0;

enum PathQueueRequestState
{
	PATHQ_STATE_INVALID,
	PATHQ_STATE_WORKING,
	PATHQ_STATE_READY,
};

typedef unsigned int PathQueueRef;

class PathQueue
{
	struct PathQuery
	{
		// Path find start and end location.
		float startPos[3], endPos[3];
		dtPolyRef startRef, endRef;
		// Result.
		dtPolyRef path[AGENT_MAX_PATH];
		bool ready;
		int npath;
		PathQueueRef ref;
		const dtQueryFilter* filter; // TODO: This is potentially dangerous!
		int keepalive;
	};
	
	static const int MAX_QUEUE = 8;
	PathQuery m_queue[MAX_QUEUE];
	PathQueueRef m_nextHandle;
	
	int m_delay;
	
public:
	PathQueue();
	~PathQueue();
	
	void update(dtNavMeshQuery* navquery);
	PathQueueRef request(dtPolyRef startRef, dtPolyRef endRef,
						 const float* startPos, const float* endPos, 
						 const dtQueryFilter* filter);
	int getRequestState(PathQueueRef ref);
	int getPathResult(PathQueueRef ref, dtPolyRef* path, const int maxPath);
};

enum MoverState
{
	MOVER_INIT,
	MOVER_OK,
	MOVER_FAILED,
};

enum MoverTargetState
{
	MOVER_TARGET_NONE,
	MOVER_TARGET_REQUESTING,
	MOVER_TARGET_WAITING_FOR_PATH,
	MOVER_TARGET_VALID,
	MOVER_TARGET_FAILED,
};

struct Mover
{
	float m_pos[3];
	float m_target[3];
	float m_radius, m_height;
	
	float m_dvel[3];
	float m_nvel[3];
	float m_vel[3];
	float m_npos[3];
	float m_disp[3];
	
	float m_pathOptimizationRange;
	
	float m_colradius;

	float m_localCenter[3];
	float m_localSegs[AGENT_MAX_LOCALSEGS*6];
	int m_localSegCount;
	
	unsigned char m_state;

	float m_reqTarget[3];
	unsigned char m_reqTargetState;
	dtPolyRef m_reqTargetRef;
	
	PathQueueRef m_pathReqRef;
	dtPolyRef m_path[AGENT_MAX_PATH];
	int m_npath;
	
	float m_cornerVerts[AGENT_MAX_CORNERS*3];
	unsigned char m_cornerFlags[AGENT_MAX_CORNERS];
	dtPolyRef m_cornerPolys[AGENT_MAX_CORNERS];
	int m_ncorners;
	
	void init(const float* p, const float r, const float h, const float cr, const float por);
	
	void requestMoveTarget(const float* pos);
	
	void updatePathState(dtNavMeshQuery* navquery, const dtQueryFilter* filter, const float* ext,
						 PathQueue* pathq);

	void updateLocalNeighbourhood(dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	void updateCorners(dtNavMeshQuery* navquery, const dtQueryFilter* filter, float* opts = 0, float* opte = 0);

	void integrate(const float maxAcc, const float dt);
	
	void updateLocation(dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	float getDistanceToGoal(const float range) const;

	void calcSmoothSteerDirection(float* dvel);
	void calcStraightSteerDirection(float* dvel);
	
	void appendLocalCollisionSegments(dtObstacleAvoidanceQuery* obstacleQuery);

	void getBounds(float* bmin, float* bmax) const
	{
	   bmin[0] = m_pos[0] - m_radius;
	   bmin[1] = m_pos[1];
	   bmin[2] = m_pos[2] - m_radius;
	   bmax[0] = m_pos[0] + m_radius;
	   bmax[1] = m_pos[1] + m_height;
	   bmax[2] = m_pos[2] + m_radius;
	}
				   
};

struct Agent
{
	unsigned char active;
	
	Mover mover;
	
	float maxspeed;
	float t;
	float var;
	
	float opts[3], opte[3];
	
	float trail[AGENT_MAX_TRAIL*3];
	int htrail;
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
	PathQueue m_pathq;
	
	int m_totalTime;
	int m_rvoTime;
	int m_sampleCount;
	
	int getNeighbours(const float* pos, const float height, const float range,
					  const Agent* skip, Agent** result, const int maxResult);

public:
	CrowdManager();
	~CrowdManager();
	
	void reset();
	const Agent* getAgent(const int idx);
	const int getAgentCount() const;
	int addAgent(const float* pos, const float radius, const float height);
	void removeAgent(const int idx);
	void requestMoveTarget(const int idx, const float* pos);
	
	int getActiveAgents(Agent** agents, const int maxAgents);
	
	void update(const float dt, unsigned int flags, dtNavMeshQuery* navquery);
	
	const dtObstacleAvoidanceDebugData* getVODebugData(const int idx) const { return m_vodebug[idx]; }	
	inline int getTotalTime() const { return m_totalTime; }
	inline int getRVOTime() const { return m_rvoTime; }
	inline int getSampleCount() const { return m_sampleCount; }
};


#endif // CROWDMANAGER_H