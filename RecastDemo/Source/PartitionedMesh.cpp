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

#include "PartitionedMesh.h"

#include <cmath>
#include <cstdlib>

struct IndexedBounds
{
	float bmin[2];
	float bmax[2];
	int index;
};

namespace
{
int compareMinX(const void* va, const void* vb)
{
	return static_cast<int>(static_cast<const IndexedBounds*>(va)->bmin[0] - static_cast<const IndexedBounds*>(vb)->bmin[0]);
}

int compareMinY(const void* va, const void* vb)
{
	return static_cast<int>(static_cast<const IndexedBounds*>(va)->bmin[1] - static_cast<const IndexedBounds*>(vb)->bmin[1]);
}

/// Calculates the total extent of all bounds in the given index range
void calcTotalBounds(const std::vector<IndexedBounds> bounds, const int start, const int end, float* outBMin, float* outBMax)
{
	outBMin[0] = bounds[start].bmin[0];
	outBMin[1] = bounds[start].bmin[1];

	outBMax[0] = bounds[start].bmax[0];
	outBMax[1] = bounds[start].bmax[1];

	for (int boundIndex = start + 1; boundIndex < end; ++boundIndex)
	{
		const IndexedBounds& it = bounds[boundIndex];
		outBMin[0] = std::min(it.bmin[0], outBMin[0]);
		outBMin[1] = std::min(it.bmin[1], outBMin[1]);

		outBMax[0] = std::max(it.bmax[0], outBMax[0]);
		outBMax[1] = std::max(it.bmax[1], outBMax[1]);
	}
}

void subdivide(
	std::vector<IndexedBounds> triBounds,
	int imin,
	int imax,
	int trisPerChunk,
	int& curNode,
	PartitionedMesh::Node* nodes,
	const int maxNodes,
	int& curTri,
	int* outTris,
	const int* inTris)
{
	const int numTriBoundsInRange = imax - imin;
	const int icur = curNode;

	if (curNode >= maxNodes)
	{
		return;
	}

	PartitionedMesh::Node& node = nodes[curNode];
	curNode++;

	if (numTriBoundsInRange <= trisPerChunk)  // Leaf
	{
		// Get total bounds of all triangles
		calcTotalBounds(triBounds, imin, imax, node.bmin, node.bmax);

		// Copy triangles.
		node.triIndex = curTri;
		node.numTris = numTriBoundsInRange;
		for (int triIndex = imin; triIndex < imax; ++triIndex)
		{
			const int* src = &inTris[triBounds[triIndex].index * 3];
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
		calcTotalBounds(triBounds, imin, imax, node.bmin, node.bmax);

		float xLength = node.bmax[0] - node.bmin[0];
		float yLength = node.bmax[1] - node.bmin[1];

		// Sort along the longest axis
		qsort(
			triBounds.data() + imin,
			static_cast<size_t>(numTriBoundsInRange),
			sizeof(IndexedBounds),
			(xLength >= yLength) ? compareMinX : compareMinY);

		int isplit = imin + numTriBoundsInRange / 2;

		// Left
		subdivide(triBounds, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		// Right
		subdivide(triBounds, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

		// Negative index means escape.
		node.triIndex = icur - curNode;
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
	float d[]{q[0] - p[0], q[1] - p[1]};

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

void PartitionedMesh::PartitionMesh(const float* verts, const int* tris, int numTris, int trisPerChunk)
{
	// Calculate the XZ bounds of every triangle.
	std::vector<IndexedBounds> triBounds;
	triBounds.resize(numTris);
	for (int triIndex = 0; triIndex < numTris; triIndex++)
	{
		const int* tri = &tris[triIndex * 3];
		IndexedBounds& bound = triBounds[triIndex];
		bound.index = triIndex;
		bound.bmin[0] = bound.bmax[0] = verts[tri[0] * 3 + 0];
		bound.bmin[1] = bound.bmax[1] = verts[tri[0] * 3 + 2];
		for (int vertIndex = 1; vertIndex < 3; ++vertIndex)
		{
			const float x = verts[tri[vertIndex] * 3 + 0];
			bound.bmin[0] = std::min(x, bound.bmin[0]);
			bound.bmax[0] = std::max(x, bound.bmax[0]);

			const float z = verts[tri[vertIndex] * 3 + 2];
			bound.bmin[1] = std::min(z, bound.bmin[1]);
			bound.bmax[1] = std::max(z, bound.bmax[1]);
		}
	}

	// Build tree
	int numChunks = static_cast<int>(ceilf(static_cast<float>(numTris) / static_cast<float>(trisPerChunk)));
	nodes.resize(numChunks * 4);
	this->tris.resize(numTris * 3);
	int curTri = 0;
	int curNode = 0;
	subdivide(triBounds, 0, numTris, trisPerChunk, curNode, nodes.data(), numChunks * 4, curTri, this->tris.data(), tris);
	nnodes = curNode;

	// Calc max tris per chunk.
	maxTrisPerChunk = 0;
	for (auto& node : nodes)
	{
		// Skip if it's not a leaf node
		if (node.triIndex < 0)
		{
			continue;
		}
		maxTrisPerChunk = std::max(maxTrisPerChunk, node.numTris);
	}
}

void PartitionedMesh::GetNodesOverlappingRect(float bmin[2], float bmax[2], std::vector<int>& outNodes) const
{
	// Traverse tree
	for (int nodeIndex = 0; nodeIndex < this->nnodes;)
	{
		const Node* node = &this->nodes[nodeIndex];
		const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->triIndex >= 0;

		if (isLeafNode && overlap)
		{
			outNodes.emplace_back(nodeIndex);
		}

		if (overlap || isLeafNode)
		{
			nodeIndex++;
		}
		else
		{
			// escape index
			nodeIndex -= node->triIndex;
		}
	}
}

void PartitionedMesh::GetNodesOverlappingSegment(float start[2], float end[2], std::vector<int>& outNodes) const
{
	// Traverse tree
	for (int nodeIndex = 0; nodeIndex < this->nnodes;)
	{
		const Node* node = &this->nodes[nodeIndex];
		const bool overlap = checkOverlapSegment(start, end, node->bmin, node->bmax);
		const bool isLeafNode = node->triIndex >= 0;

		if (isLeafNode && overlap)
		{
			outNodes.emplace_back(nodeIndex);
		}

		if (overlap || isLeafNode)
		{
			nodeIndex++;
		}
		else
		{
			// escape index
			nodeIndex -= node->triIndex;
		}
	}
}
