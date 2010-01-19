//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"

class InputGeom
{
	rcChunkyTriMesh* m_chunkyMesh;
	rcMeshLoaderObj* m_mesh;
	float m_meshBMin[3], m_meshBMax[3];
	
	static const int MAX_OFFMESH_CONNECTIONS = 256;
	
	float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*3*2];
	float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
	unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
	int m_offMeshConCount;
	
public:
	InputGeom();
	~InputGeom();
	
	bool loadMesh(const char* filepath);
	
	// Method to return static mesh data.
	inline const rcMeshLoaderObj* getMesh() const { return m_mesh; }
	inline const float* getMeshBoundsMin() const { return m_meshBMin; }
	inline const float* getMeshBoundsMax() const { return m_meshBMax; }
	inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }
	bool raycastMesh(float* src, float* dst, float& tmin);

	// Extra links
	int getOffMeshConnectionCount() const { return m_offMeshConCount; }
	const float* getOffMeshConnectionVerts() const { return m_offMeshConVerts; }
	const float* getOffMeshConnectionRads() const { return m_offMeshConRads; }
	const unsigned char* getOffMeshConnectionDirs() const { return m_offMeshConDirs; }
	void addOffMeshConnection(const float* spos, const float* epos, const float rad, unsigned char bidir);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
};

#endif // INPUTGEOM_H
