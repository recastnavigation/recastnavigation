#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H


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
	
public:
	Sample();
	virtual ~Sample();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();

	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);
	
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();

	void resetCommonSettings();
	void handleCommonSettings();
};


#endif // RECASTSAMPLE_H
