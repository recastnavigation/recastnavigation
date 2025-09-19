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

#ifndef CHUNKYTRIMESH_H
#define CHUNKYTRIMESH_H

#include "Common.h"

struct rcChunkyTriMeshNode
{
	//float bmin[2];
	//float bmax[2];
	Vector2 bmin;
	Vector2 bmax;
	int i;
	int n;
};

struct rcChunkyTriMesh
{
	inline rcChunkyTriMesh() : nodes(0), nnodes(0), maxTrisPerChunk(0) {};
	inline ~rcChunkyTriMesh() { delete [] nodes; }

	rcChunkyTriMeshNode* nodes;
	int nnodes;
	int maxTrisPerChunk;
	std::vector<Triangle> tris;
	//int* tris;
	//int ntris;
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcChunkyTriMesh(const rcChunkyTriMesh&);
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh&);
};

/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
//bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris, int trisPerChunk, rcChunkyTriMesh* cm);
bool rcCreateChunkyTriMesh(const std::vector<Vector3>& verts, const std::vector<Triangle>& tris, int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
void rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, Vector2& bmin, Vector2& bmax, std::vector<int>& ids);

/// Returns the chunk indices which overlap the input segment.
void rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, Vector2& p, Vector2& q, std::vector<int>& ids);


#endif // CHUNKYTRIMESH_H
