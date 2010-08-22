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

// Tool to create crowds.

enum BodyType
{
	BODY_CIRCLE = 0,
	BODY_SEGMENT = 1,
};

struct ObstacleBody
{
	float p[3], q[3];		// Position of the obstacle
	float vel[3];			// Velocity of the obstacle
	float dvel[3];			// Velocity of the obstacle
	float rad;				// Radius of the obstacle
	float dp[3], np[3];		// Use for side selection during sampling.
	bool touch;
	int type;				// Type of the obstacle (see ObstacleType)
};


static const int RVO_SAMPLE_RAD = 15;
static const int MAX_RVO_SAMPLES = (RVO_SAMPLE_RAD*2+1)*(RVO_SAMPLE_RAD*2+1) + 100;

struct RVODebugData
{
	inline RVODebugData() : ns(0) {}
	float spos[MAX_RVO_SAMPLES*3];
	float scs[MAX_RVO_SAMPLES];
	float spen[MAX_RVO_SAMPLES];
	float svpen[MAX_RVO_SAMPLES];
	float svcpen[MAX_RVO_SAMPLES];
	float sspen[MAX_RVO_SAMPLES];
	float stpen[MAX_RVO_SAMPLES];
	
	int ns;
};

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

	RVODebugData rvo;
	
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


class SampleGraph
{
	static const int MAX_SAMPLES = 512;
	float m_samples[MAX_SAMPLES];
	int m_hsamples;
public:
	SampleGraph() :
		m_hsamples(0)
	{
		for (int i = 0; i < MAX_SAMPLES; ++i)
			m_samples[i] = 0;
	}
	
	~SampleGraph()
	{
	}
	
	inline void addSample(const float val)
	{
		m_hsamples = (m_hsamples+MAX_SAMPLES-1) % MAX_SAMPLES;
		m_samples[m_hsamples] = val;
	}
	
	inline int getSampleCount() const
	{
		return MAX_SAMPLES;
	}
	
	inline float getSample(const int i) const
	{
		return m_samples[(m_hsamples+i) % MAX_SAMPLES];
	}
	
	inline float getSampleMin() const
	{
		float val = m_samples[0];
		for (int i = 1; i < MAX_SAMPLES; ++i)
			if (m_samples[i] < val)
				val = m_samples[i];
		return val;
	} 
	
	inline float getSampleMax() const
	{
		float val = m_samples[0];
		for (int i = 1; i < MAX_SAMPLES; ++i)
			if (m_samples[i] > val)
				val = m_samples[i];
		return val;
	} 
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
	int m_shortcutIter;

	SampleGraph m_totalTime;
	SampleGraph m_rvoTime;

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

	const SampleGraph* getTotalTimeGraph() const { return &m_totalTime; }
	const SampleGraph* getRVOTimeGraph() const { return &m_rvoTime; }
};

class CrowdTool : public SampleTool
{
	Sample* m_sample;
	float m_targetPos[3];
	bool m_targetPosSet;
	
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
