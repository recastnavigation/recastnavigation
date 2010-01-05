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

struct ExtraLink
{
	float spos[3];	// Start position of the link.
	float epos[3];	// End position of the link.
};

class InputGeom
{
	rcChunkyTriMesh* m_chunkyMesh;
	rcMeshLoaderObj* m_mesh;
	float m_meshBMin[3], m_meshBMax[3];
	
	static const int MAX_LINKS = 256;
	
	ExtraLink m_links[MAX_LINKS];
	int m_nlinks;
	
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
	int getExtraLinkCount() const { return m_nlinks; }
	ExtraLink* getExtraLink(int i) { return &m_links[i]; }
	void addExtraLink(const float* spos, const float* epos);
	void deleteExtraLink(int i);
};

#endif // INPUTGEOM_H
