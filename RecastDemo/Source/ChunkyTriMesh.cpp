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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

struct IndexedBounds
{
	float bmin[2];
	float bmax[2];
	int i;
};

namespace
{
int compareMinX(const void* va, const void* vb)
{
	return ((const IndexedBounds*)va)->bmin[0] - ((const IndexedBounds*)vb)->bmin[0];
}

int compareMinY(const void* va, const void* vb)
{
	return ((const IndexedBounds*)va)->bmin[1] - ((const IndexedBounds*)vb)->bmin[1];
}

/// Calculates the total extent of all bounds in the given index range
void calcTotalBounds(const IndexedBounds* items, const int startIndex, const int endIndex, float* outBMin, float* outBMax)
{
	outBMin[0] = items[startIndex].bmin[0];
	outBMin[1] = items[startIndex].bmin[1];

	outBMax[0] = items[startIndex].bmax[0];
	outBMax[1] = items[startIndex].bmax[1];

	for (int i = startIndex + 1; i < endIndex; ++i)
	{
		const IndexedBounds& it = items[i];
		if (it.bmin[0] < outBMin[0])
		{
			outBMin[0] = it.bmin[0];
		}
		if (it.bmin[1] < outBMin[1])
		{
			outBMin[1] = it.bmin[1];
		}

		if (it.bmax[0] > outBMax[0])
		{
			outBMax[0] = it.bmax[0];
		}
		if (it.bmax[1] > outBMax[1])
		{
			outBMax[1] = it.bmax[1];
		}
	}
}

void subdivide(
	IndexedBounds* items,
	int nitems,
	int imin,
	int imax,
	int trisPerChunk,
	int& curNode,
	ChunkyTriMesh::Node* nodes,
	const int maxNodes,
	int& curTri,
	int* outTris,
	const int* inTris)
{
	int inum = imax - imin;
	int icur = curNode;

	if (curNode >= maxNodes)
	{
		return;
	}

	ChunkyTriMesh::Node& node = nodes[curNode++];

	if (inum <= trisPerChunk)
	{
		// Leaf
		calcTotalBounds(items, imin, imax, node.bmin, node.bmax);

		// Copy triangles.
		node.i = curTri;
		node.n = inum;

		for (int i = imin; i < imax; ++i)
		{
			const int* src = &inTris[items[i].i * 3];
			int* dst = &outTris[curTri * 3];
			curTri++;
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
		}
	}
	else
	{
		// Split
		calcTotalBounds(items, imin, imax, node.bmin, node.bmax);

		float xLength = node.bmax[0] - node.bmin[0];
		float yLength = node.bmax[1] - node.bmin[1];

		// Sort along the longest axis
		qsort(items + imin, static_cast<size_t>(inum), sizeof(IndexedBounds), (xLength >= yLength) ? compareMinX : compareMinY);

		int isplit = imin + inum / 2;

		// Left
		subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		// Right
		subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

bool checkOverlapRect(const float amin[2], const float amax[2], const float bmin[2], const float bmax[2])
{
	return amin[0] <= bmax[0] && amax[0] >= bmin[0] && amin[1] <= bmax[1] && amax[1] >= bmin[1];
}

bool checkOverlapSegment(const float p[2], const float q[2], const float bmin[2], const float bmax[2])
{
	float tmin = 0;
	float tmax = 1;
	float d[] {
		q[0] - p[0],
		q[1] - p[1]
	};

	for (int i = 0; i < 2; i++)
	{
		static const float EPSILON = 1e-6f;
		if (fabsf(d[i]) < EPSILON)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (p[i] < bmin[i] || p[i] > bmax[i])
			{
				return false;
			}
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			float ood = 1.0f / d[i];
			float t1 = (bmin[i] - p[i]) * ood;
			float t2 = (bmax[i] - p[i]) * ood;
			if (t1 > t2)
			{
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > tmin)
			{
				tmin = t1;
			}
			if (t2 < tmax)
			{
				tmax = t2;
			}
			if (tmin > tmax)
			{
				return false;
			}
		}
	}
	return true;
}

}

bool ChunkyTriMesh::TryPartitionMesh(const float* verts, const int* tris, int ntris, int trisPerChunk)
{
	int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

	nodes.resize(nchunks * 4);
	this->tris.resize(ntris * 3);

	// Build tree
	IndexedBounds* items = new IndexedBounds[ntris];
	if (!items)
	{
		return false;
	}

	for (int i = 0; i < ntris; i++)
	{
		const int* t = &tris[i * 3];
		IndexedBounds& it = items[i];
		it.i = i;
		// Calc triangle XZ bounds.
		it.bmin[0] = it.bmax[0] = verts[t[0] * 3 + 0];
		it.bmin[1] = it.bmax[1] = verts[t[0] * 3 + 2];
		for (int j = 1; j < 3; ++j)
		{
			const float* v = &verts[t[j] * 3];
			if (v[0] < it.bmin[0])
			{
				it.bmin[0] = v[0];
			}
			if (v[2] < it.bmin[1])
			{
				it.bmin[1] = v[2];
			}

			if (v[0] > it.bmax[0])
			{
				it.bmax[0] = v[0];
			}
			if (v[2] > it.bmax[1])
			{
				it.bmax[1] = v[2];
			}
		}
	}

	int curTri = 0;
	int curNode = 0;
	subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, nodes.data(), nchunks * 4, curTri, this->tris.data(), tris);

	delete[] items;

	nnodes = curNode;

	// Calc max tris per node.
	maxTrisPerChunk = 0;
	for (int i = 0; i < nnodes; ++i)
	{
		Node& node = nodes[i];
		const bool isLeaf = node.i >= 0;
		if (!isLeaf)
		{
			continue;
		}
		if (node.n > maxTrisPerChunk)
		{
			maxTrisPerChunk = node.n;
		}
	}

	return true;
}

int ChunkyTriMesh::GetChunksOverlappingRect(float bmin[2], float bmax[2], int* ids, const int maxIds) const
{
	// Traverse tree
	int i = 0;
	int n = 0;
	while (i < this->nnodes)
	{
		const Node* node = &this->nodes[i];
		const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;

		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}

		if (overlap || isLeafNode)
		{
			i++;
		}
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}

	return n;
}

int ChunkyTriMesh::GetChunksOverlappingSegment(float segmentStart[2], float segmentEnd[2], int* ids, const int maxIds) const
{
	// Traverse tree
	int i = 0;
	int n = 0;
	while (i < this->nnodes)
	{
		const Node* node = &this->nodes[i];
		const bool overlap = checkOverlapSegment(segmentStart, segmentEnd, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;

		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}

		if (overlap || isLeafNode)
		{
			i++;
		}
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}

	return n;
}
