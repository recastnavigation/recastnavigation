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

#ifndef DETOUTPATHCORRIDOR_H
#define DETOUTPATHCORRIDOR_H

#include "DetourNavMeshQuery.h"


class dtPathCorridor
{
	float m_pos[3];
	float m_target[3];
	
	dtPolyRef* m_path;
	int m_npath;
	int m_maxPath;
	
public:
	dtPathCorridor();
	~dtPathCorridor();
	
	bool init(const int maxPath);
	
	void reset(dtPolyRef ref, const float* pos);
	
	int findCorners(float* cornerVerts, unsigned char* cornerFlags,
					dtPolyRef* cornerPolys, const int maxCorners,
					dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	void optimizePathVisibility(const float* next, const float pathOptimizationRange,
								dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	bool optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	bool moveOverOffmeshConnection(dtPolyRef offMeshConRef, dtPolyRef* refs,
								   float* startPos, float* endPos,
								   dtNavMeshQuery* navquery);
	
	void movePosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	void moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	void setCorridor(const float* target, const dtPolyRef* polys, const int npolys);
	
	inline const float* getPos() const { return m_pos; }	
	inline const float* getTarget() const { return m_target; }
	
	inline dtPolyRef getFirstPoly() const { return m_npath ? m_path[0] : 0; }
	
	inline const dtPolyRef* getPath() const { return m_path; }
	inline int getPathCount() const { return m_npath; } 	
};

int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
							  const dtPolyRef* visited, const int nvisited);

int dtMergeCorridorEndMoved(dtPolyRef* path, const int npath, const int maxPath,
							const dtPolyRef* visited, const int nvisited);

int dtMergeCorridorStartShortcut(dtPolyRef* path, const int npath, const int maxPath,
								 const dtPolyRef* visited, const int nvisited);

#endif // DETOUTPATHCORRIDOR_H
