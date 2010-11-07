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


class ProximityGrid
{
	int m_maxItems;
	float m_cellSize;
	float m_invCellSize;

	struct Item
	{
		unsigned short id;
		short x,y;
		unsigned short next;
	};
	Item* m_pool;
	int m_poolHead;
	int m_poolSize;
	
	unsigned short* m_buckets;
	int m_bucketsSize;
	
	int m_bounds[4];
	
public:
	ProximityGrid();
	~ProximityGrid();
	
	bool init(const int maxItems, const float cellSize);
	
	void clear();
	
	void addItem(const unsigned short id,
				 const float minx, const float miny,
				 const float maxx, const float maxy);
	
	int queryItems(const float minx, const float miny,
				   const float maxx, const float maxy,
				   unsigned short* ids, const int maxIds) const;
	
	int getItemCountAt(const int x, const int y) const;
	const int* getBounds() const { return m_bounds; }
	const float getCellSize() const { return m_cellSize; }
};




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
	static const int PQ_MAX_PATH = 256;
	
	struct PathQuery
	{
		// Path find start and end location.
		float startPos[3], endPos[3];
		dtPolyRef startRef, endRef;
		// Result.
		dtPolyRef path[PQ_MAX_PATH];
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


class PathCorridor
{
	float m_pos[3];
	float m_target[3];

	dtPolyRef* m_path;
	int m_npath;
	int m_maxPath;
	
public:
	PathCorridor();
	~PathCorridor();
	
	bool init(const int maxPath);
	
	void reset(dtPolyRef ref, const float* pos);
	
	int findCorners(float* cornerVerts, unsigned char* cornerFlags,
					dtPolyRef* cornerPolys, const int maxCorners,
					dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	void optimizePathVisibility(const float* next, const float pathOptimizationRange,
								dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	bool optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	void movePosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	void moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	void setCorridor(const float* target, const dtPolyRef* polys, const int npolys);

	inline const float* getPos() const { return m_pos; }	
	inline const float* getTarget() const { return m_target; }
	
	inline dtPolyRef getFirstPoly() const { return m_npath ? m_path[0] : 0; }
	
	inline const dtPolyRef* getPath() const { return m_path; }
	inline int getPathCount() const { return m_npath; } 	
};


class LocalBoundary
{
	static const int MAX_SEGS = 8;
	
	struct Segment
	{
		float s[6];	// Segment start/end
		float d;	// Distance for pruning.
	};
	
	float m_center[3];
	Segment m_segs[MAX_SEGS];
	int m_nsegs;
	
	void addSegment(const float dist, const float* seg);

public:
	LocalBoundary();
	~LocalBoundary();

	void reset();
	
	void update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
				dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	inline const float* getCenter() const { return m_center; }
	inline int getSegmentCount() const { return m_nsegs; }
	inline const float* getSegment(int i) const { return m_segs[i].s; }
};


static const int AGENT_MAX_NEIGHBOURS = 6;
static const int AGENT_MAX_CORNERS = 4;
static const int AGENT_MAX_TRAIL = 64;

struct Neighbour
{
	int idx;
	float dist;
};

struct Agent
{
	void integrate(const float maxAcc, const float dt);
	void calcSmoothSteerDirection(float* dir);
	void calcStraightSteerDirection(float* dir);
	float getDistanceToGoal(const float range) const;
	
	unsigned char active;
	
	PathCorridor corridor;
	LocalBoundary boundary;
	
	float maxspeed;
	float t;
	float var;

	float collisionQueryRange;
	float pathOptimizationRange;

	float topologyOptTime;
	
	Neighbour neis[AGENT_MAX_NEIGHBOURS];
	int nneis;
	
	float radius, height;
	float npos[3];
	float disp[3];
	float dvel[3];
	float nvel[3];
	float vel[3];
	
	float cornerVerts[AGENT_MAX_CORNERS*3];
	unsigned char cornerFlags[AGENT_MAX_CORNERS];
	dtPolyRef cornerPolys[AGENT_MAX_CORNERS];
	int ncorners;
	
	float opts[3], opte[3];
	
	float trail[AGENT_MAX_TRAIL*3];
	int htrail;
};


enum UpdateFlags
{
	CROWDMAN_ANTICIPATE_TURNS = 1,
	CROWDMAN_USE_VO = 2,
	CROWDMAN_DRUNK = 4,
	CROWDMAN_OPTIMIZE_VIS = 8,
	CROWDMAN_OPTIMIZE_TOPO = 16,
};


class CrowdManager
{
	static const int MAX_AGENTS = 128;
	Agent m_agents[MAX_AGENTS];
	dtObstacleAvoidanceDebugData* m_vodebug[MAX_AGENTS];
	
	dtObstacleAvoidanceQuery* m_obstacleQuery;
	PathQueue m_pathq;
	ProximityGrid m_grid;
	
	dtPolyRef* m_pathResult;
	int m_maxPathResult;
	
	float m_ext[3];
	dtQueryFilter m_filter;

	int m_totalTime;
	int m_rvoTime;
	int m_sampleCount;

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
		unsigned char state;			// State of the request
		int idx;						// Agent index
		dtPolyRef ref;					// Goal ref
		float pos[3];					// Goal position
		PathQueueRef pathqRef;			// Path find query ref
		dtPolyRef aref;					// Goal adjustment ref
		float apos[3];					// Goal adjustment pos
		dtPolyRef temp[MAX_TEMP_PATH];	// Adjusted path to the goal
		int ntemp;
	};
	MoveRequest m_moveRequests[MAX_AGENTS];
	int m_moveRequestCount;
	
	int getNeighbours(const float* pos, const float height, const float range,
					  const Agent* skip, Neighbour* result, const int maxResult);

	void updateTopologyOptimization(const float dt, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	void updateMoveRequest(const float dt, dtNavMeshQuery* navquery, const dtQueryFilter* filter);

public:
	CrowdManager();
	~CrowdManager();
	
	void reset();
	const Agent* getAgent(const int idx);
	const int getAgentCount() const;
	int addAgent(const float* pos, const float radius, const float height, dtNavMeshQuery* navquery);
	void removeAgent(const int idx);
	
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);
	bool adjustMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	int getActiveAgents(Agent** agents, const int maxAgents);
	void update(const float dt, unsigned int flags, dtNavMeshQuery* navquery);
	
	const dtQueryFilter* getFilter() const { return &m_filter; }
	const float* getQueryExtents() const { return m_ext; }
	
	const dtObstacleAvoidanceDebugData* getVODebugData(const int idx) const { return m_vodebug[idx]; }	
	inline int getTotalTime() const { return m_totalTime; }
	inline int getRVOTime() const { return m_rvoTime; }
	inline int getSampleCount() const { return m_sampleCount; }
	const ProximityGrid* getGrid() const { return &m_grid; }

};


#endif // CROWDMANAGER_H