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

#include "ChunkyTriMesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

struct BoundsItem
{
	//float bmin[2];
	//float bmax[2];
	Vector2 bmin;
	Vector2 bmax;
	int i; // 第几个三角面
};

static int compareItemX(const void* va, const void* vb)
{
	const BoundsItem* a = (const BoundsItem*)va;
	const BoundsItem* b = (const BoundsItem*)vb;
	if (a->bmin.x < b->bmin.x)
		return -1;
	if (a->bmin.x > b->bmin.x)
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BoundsItem* a = (const BoundsItem*)va;
	const BoundsItem* b = (const BoundsItem*)vb;
	if (a->bmin.y < b->bmin.y)
		return -1;
	if (a->bmin.y > b->bmin.y)
		return 1;
	return 0;
}

static void calcExtends(const BoundsItem* items, const int /*nitems*/,
						const int imin, const int imax,
						Vector2& bmin, Vector2& bmax)
{
	bmin.x = items[imin].bmin.x;
	bmin.y = items[imin].bmin.y;
	
	bmax.x = items[imin].bmax.x;
	bmax.y = items[imin].bmax.y;
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BoundsItem& it = items[i];
		if (it.bmin.x < bmin.x) bmin.x = it.bmin.x;
		if (it.bmin.y < bmin.y) bmin.y = it.bmin.y;
		
		if (it.bmax.x > bmax.x) bmax.x = it.bmax.x;
		if (it.bmax.y > bmax.y) bmax.y = it.bmax.y;
	}

	//bmin[0] = items[imin].bmin[0];
	//bmin[1] = items[imin].bmin[1];

	//bmax[0] = items[imin].bmax[0];
	//bmax[1] = items[imin].bmax[1];

	//for (int i = imin + 1; i < imax; ++i)
	//{
	//	const BoundsItem& it = items[i];
	//	if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
	//	if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];

	//	if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
	//	if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
	//}
}

inline int longestAxis(float x, float y)
{
	return y > x ? 1 : 0;
}

static void subdivide(BoundsItem* items, int nitems, int imin, int imax, int trisPerChunk,
					  int& curNode, rcChunkyTriMeshNode* nodes, const int maxNodes,
					  int& curTri, std::vector<Triangle>& outTris, const std::vector<Triangle>& inTris)
{
	int inum = imax - imin;
	int icur = curNode;
	
	if (curNode > maxNodes)
		return;

	rcChunkyTriMeshNode& node = nodes[curNode++];
	
	if (inum <= trisPerChunk)
	{
		// Leaf
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		// Copy triangles.
		node.i = curTri;
		node.n = inum;
		
		for (int i = imin; i < imax; ++i)
		{
			const auto& src = inTris[items[i].i];
			auto& dst = outTris[curTri];
			
			curTri++;

			//Triangle dst;
			dst.v0 = src.v0;
			dst.v1 = src.v1;
			dst.v2 = src.v2;
			//node.tris.push_back(dst);
		}
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax.x - node.bmin.x,
							   node.bmax.y - node.bmin.y);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemY);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		// Right
		subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

//bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
//						   int trisPerChunk, rcChunkyTriMesh* cm)
//{
//	int nchunks = (ntris + trisPerChunk-1) / trisPerChunk;
//
//	cm->nodes = new rcChunkyTriMeshNode[nchunks*4];
//	if (!cm->nodes)
//		return false;
//		
//	cm->tris = new int[ntris*3];
//	if (!cm->tris)
//		return false;
//		
//	cm->ntris = ntris;
//
//	// Build tree
//	BoundsItem* items = new BoundsItem[ntris];
//	if (!items)
//		return false;
//
//	for (int i = 0; i < ntris; i++)
//	{
//		const int* t = &tris[i*3];
//		BoundsItem& it = items[i];
//		it.i = i;
//		// Calc triangle XZ bounds.
//		it.bmin.x = it.bmax.x = verts[t[0]*3+0];
//		it.bmin.y = it.bmax.y = verts[t[0]*3+2];
//		for (int j = 1; j < 3; ++j)
//		{
//			const float* v = &verts[t[j]*3];
//			if (v[0] < it.bmin.x) it.bmin.x = v[0]; 
//			if (v[2] < it.bmin.y) it.bmin.y = v[2]; 
//
//			if (v[0] > it.bmax.x) it.bmax.x = v[0]; 
//			if (v[2] > it.bmax.y) it.bmax.y = v[2]; 
//		}
//	}
//
//	int curTri = 0;
//	int curNode = 0;
//	subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks*4, curTri, cm->tris, tris);
//	
//	delete [] items;
//	
//	cm->nnodes = curNode;
//	
//	// Calc max tris per node.
//	cm->maxTrisPerChunk = 0;
//	for (int i = 0; i < cm->nnodes; ++i)
//	{
//		rcChunkyTriMeshNode& node = cm->nodes[i];
//		const bool isLeaf = node.i >= 0;
//		if (!isLeaf) continue;
//		if (node.n > cm->maxTrisPerChunk)
//			cm->maxTrisPerChunk = node.n;
//	}
//	 
//	return true;
//}

bool rcCreateChunkyTriMesh(const std::vector<Vector3>& verts, const std::vector<Triangle>& tris, int trisPerChunk, rcChunkyTriMesh* cm)
{
	int nchunks = (int)(tris.size() + trisPerChunk - 1) / trisPerChunk;

	cm->nodes = new rcChunkyTriMeshNode[nchunks * 4];
	if (!cm->nodes)
		return false;

	//cm->tris = new int[ntris * 3];
	//if (!cm->tris)
	//	return false;

	//cm->ntris = ntris;
	cm->tris.resize(tris.size());

	// Build tree
	BoundsItem* items = new BoundsItem[tris.size()];
	if (!items)
		return false;

	for (size_t i = 0; i < tris.size(); i++)
	{
		const auto& t = tris[i];
		//const int* t = &tris[i * 3];
		BoundsItem& it = items[i];
		it.i = i;

		// Calc triangle XZ bounds.
		const auto& v0 = verts[t.v0];
		it.bmin.x = it.bmax.x = v0.x;
		it.bmin.y = it.bmax.y = v0.z;
		
		const auto& v1 = verts[t.v1];
		if (v1.x < it.bmin.x) it.bmin.x = v1.x;
		if (v1.z < it.bmin.y) it.bmin.y = v1.z;
		if (v1.x > it.bmax.x) it.bmax.x = v1.x;
		if (v1.z > it.bmax.y) it.bmax.y = v1.z;

		const auto& v2 = verts[t.v2];
		if (v2.x < it.bmin.x) it.bmin.x = v2.x;
		if (v2.z < it.bmin.y) it.bmin.y = v2.z;
		if (v2.x > it.bmax.x) it.bmax.x = v2.x;
		if (v2.z > it.bmax.y) it.bmax.y = v2.z;

		//it.bmin.x = it.bmax.x = verts[t[0] * 3 + 0];
		//it.bmin.y = it.bmax.y = verts[t[0] * 3 + 2];
		//for (int j = 1; j < 3; ++j)
		//{
		//	const float* v = &verts[t[j] * 3];
		//	if (v[0] < it.bmin.x) it.bmin.x = v[0];
		//	if (v[2] < it.bmin.y) it.bmin.y = v[2];

		//	if (v[0] > it.bmax.x) it.bmax.x = v[0];
		//	if (v[2] > it.bmax.y) it.bmax.y = v[2];
		//}
	}

	int curTri = 0;
	int curNode = 0;
	subdivide(items, (int)verts.size(), 0, (int)tris.size(), trisPerChunk, curNode, cm->nodes, nchunks * 4, curTri, cm->tris, tris);

	delete[] items;

	cm->nnodes = curNode;

	// Calc max tris per node.
	cm->maxTrisPerChunk = 0;
	for (int i = 0; i < cm->nnodes; ++i)
	{
		rcChunkyTriMeshNode& node = cm->nodes[i];
		const bool isLeaf = node.i >= 0;
		if (!isLeaf) continue;
		if (node.n > cm->maxTrisPerChunk)
			cm->maxTrisPerChunk = node.n;
	}

	return true;
}

inline bool checkOverlapRect(const float amin[2], const float amax[2],
	const float bmin[2], const float bmax[2])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	return overlap;
}

inline bool checkOverlapRect(const Vector2& amin, const Vector2& amax,
							 const Vector2& bmin, const Vector2& bmax)
{
	bool overlap = true;
	overlap = (amin.x > bmax.x || amax.x < bmin.x) ? false : overlap;
	overlap = (amin.y > bmax.y || amax.y < bmin.y) ? false : overlap;
	return overlap;
}

void rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, Vector2& bmin, Vector2& bmax, std::vector<int>& ids)
{
	// Traverse tree
	int i = 0;
	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;
		
		if (isLeafNode && overlap)
		{
			ids.push_back(i);
		}
		
		if (overlap || isLeafNode)
			i++;
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}
}



static bool checkOverlapSegment(const Vector2& p, const Vector2& q,
								const Vector2& bmin, const Vector2& bmax)
{
	static const float EPSILON = 1e-6f;

	float tmin = 0;
	float tmax = 1;

	Vector2 d;
	d.x = q.x - p.x;
	d.y = q.y - p.y;

	// x
	if (fabsf(d.x) < EPSILON)
	{
		// Ray is parallel to slab. No hit if origin not within slab
		if (p.x < bmin.x || p.x > bmax.x)
			return false;
	}
	else
	{
		// Compute intersection t value of ray with near and far plane of slab
		float ood = 1.0f / d.x;
		float t1 = (bmin.x - p.x) * ood;
		float t2 = (bmax.x - p.x) * ood;
		if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
	}

	// y
	if (fabsf(d.y) < EPSILON)
	{
		// Ray is parallel to slab. No hit if origin not within slab
		if (p.y < bmin.y || p.y > bmax.y)
			return false;
	}
	else
	{
		// Compute intersection t value of ray with near and far plane of slab
		float ood = 1.0f / d.y;
		float t1 = (bmin.y - p.y) * ood;
		float t2 = (bmax.y - p.y) * ood;
		if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
	}


	//float d[2];
	//d[0] = q[0] - p[0];
	//d[1] = q[1] - p[1];
	
	//for (int i = 0; i < 2; i++)
	//{
	//	if (fabsf(d[i]) < EPSILON)
	//	{
	//		// Ray is parallel to slab. No hit if origin not within slab
	//		if (p[i] < bmin[i] || p[i] > bmax[i])
	//			return false;
	//	}
	//	else
	//	{
	//		// Compute intersection t value of ray with near and far plane of slab
	//		float ood = 1.0f / d[i];
	//		float t1 = (bmin[i] - p[i]) * ood;
	//		float t2 = (bmax[i] - p[i]) * ood;
	//		if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
	//		if (t1 > tmin) tmin = t1;
	//		if (t2 < tmax) tmax = t2;
	//		if (tmin > tmax) return false;
	//	}
	//}
	return true;
}

void rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, Vector2& p, Vector2& q, std::vector<int>& ids)
{
	// Traverse tree
	int i = 0;
	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		const bool overlap = checkOverlapSegment(p, q, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;
		
		if (isLeafNode && overlap)
		{
			ids.push_back(i);
		}
		
		if (overlap || isLeafNode)
			i++;
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}
}
