#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H

#include "DebugDraw.h"

struct DebugDrawGL : public duDebugDraw
{
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void end();
};

enum SampleToolType
{
	TOOL_NONE = 0,
	TOOL_TILE_EDIT,
	TOOL_NAVMESH_TESTER,
};

struct SampleTool
{
	virtual ~SampleTool() {}
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
};


class Sample
{
protected:
	const float* m_verts;
	int m_nverts;
	const int* m_tris;
	const float* m_trinorms;
	int m_ntris;
	float m_bmin[3], m_bmax[3];

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	
	SampleTool* m_tool;
	
public:
	Sample();
	virtual ~Sample();
	
	void setTool(SampleTool* tool);
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* p, bool shift);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();

	virtual class dtNavMesh* getNavMesh() { return 0; }
	virtual float getAgentRadius() { return m_agentRadius; }
	virtual float getAgentHeight() { return m_agentHeight; }

	void resetCommonSettings();
	void handleCommonSettings();
};


#endif // RECASTSAMPLE_H
