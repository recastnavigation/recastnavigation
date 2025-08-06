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

#pragma once

#include <vector>

/// A spatially-partitioned mesh (k/d tree),
/// where each node contains at max trisPerChunk triangles.
struct PartitionedMesh
{
	struct Node
	{
		// xy bounds
		float bmin[2];
		float bmax[2];

		int triIndex;
		int numTris;
	};

	std::vector<Node> nodes{};
	int nnodes = 0;

	std::vector<int> tris{};
	int maxTrisPerChunk = 0;

	void PartitionMesh(const float* verts, const int* tris, int ntris, int trisPerChunk);

	/// Finds the chunk indices that overlap the input rectangle.
	void GetNodesOverlappingRect(float bmin[2], float bmax[2], std::vector<int>& outNodes) const;

	/// Returns the chunk indices which overlap the input segment.
	void GetNodesOverlappingSegment(float start[2], float end[2], std::vector<int>& outNodes) const;
};
