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

#ifndef DETOURCROWD_H
#define DETOURCROWD_H

#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourLocalBoundary.h"
#include "DetourPathCorridor.h"
#include "DetourProximityGrid.h"
#include "DetourPathQueue.h"


static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;
static const int DT_CROWDAGENT_MAX_CORNERS = 4;
static const int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;
struct dtCrowdNeighbour
{
	int idx;
	float dist;
};

enum CrowdAgentState
{
	DT_CROWDAGENT_STATE_WALKING,
	DT_CROWDAGENT_STATE_OFFMESH,
};

struct dtCrowdAgentParams
{
	float radius;
	float height;
	float maxAcceleration;
	float maxSpeed;
	float collisionQueryRange;
	float pathOptimizationRange;
	float separationWeight;
	unsigned char updateFlags;
	unsigned char obstacleAvoidanceType;
	void* userData;
};

struct dtCrowdAgent
{
	unsigned char active;
	unsigned char state;

	dtPathCorridor corridor;
	dtLocalBoundary boundary;
	
	float t;
	float var;

	float topologyOptTime;
	
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];
	int nneis;
	
	float desiredSpeed;

	float npos[3];
	float disp[3];
	float dvel[3];
	float nvel[3];
	float vel[3];

	dtCrowdAgentParams params;

	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS*3];
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];
	int ncorners;
};

struct dtCrowdAgentAnimation
{
	unsigned char active;
	float initPos[3], startPos[3], endPos[3];
	dtPolyRef polyRef;
	float t, tmax;
};

enum UpdateFlags
{
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	DT_CROWD_OPTIMIZE_VIS = 8,
	DT_CROWD_OPTIMIZE_TOPO = 16,
};

struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	dtObstacleAvoidanceDebugData* vod;
};

class dtCrowd
{
	int m_maxAgents;
	dtCrowdAgent* m_agents;
	dtCrowdAgent** m_activeAgents;
	dtCrowdAgentAnimation* m_agentAnims;
	
	dtPathQueue m_pathq;

	dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
	dtObstacleAvoidanceQuery* m_obstacleQuery;
	
	dtProximityGrid* m_grid;
	
	dtPolyRef* m_pathResult;
	int m_maxPathResult;
	
	float m_ext[3];
	dtQueryFilter m_filter;
	
	float m_maxAgentRadius;

	int m_velocitySampleCount;

	enum MoveRequestState
	{
		MR_TARGET_FAILED,
		MR_TARGET_VALID,
		MR_TARGET_REQUESTING,
		MR_TARGET_WAITING_FOR_PATH,
		MR_TARGET_ADJUST,
	};
	
	static const int MAX_TEMP_PATH = 32;

	struct MoveRequest
	{
		unsigned char state;			///< State of the request
		int idx;						///< Agent index
		dtPolyRef ref;					///< Goal ref
		float pos[3];					///< Goal position
		dtPathQueueRef pathqRef;		///< Path find query ref
		dtPolyRef aref;					///< Goal adjustment ref
		float apos[3];					///< Goal adjustment pos
		dtPolyRef temp[MAX_TEMP_PATH];	///< Adjusted path to the goal
		int ntemp;
	};
	MoveRequest* m_moveRequests;
	int m_moveRequestCount;
	
	dtNavMeshQuery* m_navquery;

	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);
	void updateMoveRequest(const float dt);

	inline int getAgentIndex(const dtCrowdAgent* agent) const  { return agent - m_agents; }
	
	void purge();
	
public:
	dtCrowd();
	~dtCrowd();
	
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);
	
	void setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params);
	const dtObstacleAvoidanceParams* getObstacleAvoidanceParams(const int idx) const;
	
	const dtCrowdAgent* getAgent(const int idx);
	const int getAgentCount() const;
	
	int addAgent(const float* pos, const dtCrowdAgentParams* params);
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);
	void removeAgent(const int idx);
	
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);
	bool adjustMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);
	void update(const float dt, dtCrowdAgentDebugInfo* debug);
	
	const dtQueryFilter* getFilter() const { return &m_filter; }
	const float* getQueryExtents() const { return m_ext; }
	
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }
	
	const dtProximityGrid* getGrid() const { return m_grid; }
	const dtPathQueue* getPathQueue() const { return &m_pathq; }
	const dtNavMeshQuery* getNavMeshQuery() const { return m_navquery; }
};


#endif // CROWDMANAGER_H
