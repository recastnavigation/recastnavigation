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
#include "CrowdManager.h"

// Tool to create crowds.

class CrowdTool : public SampleTool
{
	Sample* m_sample;
	unsigned char m_oldFlags;
	
	float m_targetPos[3];
	dtPolyRef m_targetRef;
	
	bool m_expandDebugDraw;
	bool m_showLabels;
	bool m_showCorners;
	bool m_showTargets;
	bool m_showCollisionSegments;
	bool m_showPath;
	bool m_showVO;
	bool m_showOpt;
	bool m_showGrid;
	bool m_showNodes;
	bool m_showPerfGraph;
	
	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_optimizeVis;
	bool m_optimizeTopo;
	bool m_useVO;
	bool m_drunkMove;
	
	bool m_run;
	
	CrowdManager m_crowd;
		
	ValueHistory m_crowdTotalTime;
	ValueHistory m_crowdRvoTime;
	ValueHistory m_crowdSampleCount;
	
	enum ToolMode
	{
		TOOLMODE_CREATE,
		TOOLMODE_MOVE_TARGET,
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
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
};

#endif // CROWDTOOL_H
