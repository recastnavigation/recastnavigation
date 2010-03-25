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

#ifndef DETOURNAVMESHBUILDER_H
#define DETOURNAVMESHBUILDER_H

struct dtNavMeshCreateParams
{
	// Navmesh vertices.
	const unsigned short* verts;
	int vertCount;
	// Navmesh polygons
	const unsigned short* polys;
	const unsigned short* polyFlags;
	const unsigned char* polyAreas;
	int polyCount;
	int nvp;
	// Navmesh Detail
	const unsigned short* detailMeshes;
	const float* detailVerts;
	int detailVertsCount;
	const unsigned char* detailTris;
	int detailTriCount; 
	// Off-Mesh Connections.
	const float* offMeshConVerts;
	const float* offMeshConRad;
	const unsigned short* offMeshConFlags;
	const unsigned char* offMeshConAreas;
	const unsigned char* offMeshConDir;
	int offMeshConCount;
	// Tile location
	unsigned int userId;
	int tileX, tileY;
	float bmin[3], bmax[3];
	// Settings
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float cs;
	float ch;
	int tileSize;
};

// Build navmesh data from given input data.
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);

// Swaps endianess of navmesh header.
bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int dataSize);

// Swaps endianess of the navmesh data. This function assumes that the header is in correct
// endianess already. Call dtNavMeshHeaderSwapEndian() first on the data if the data is
// assumed to be in wrong endianess to start with. If converting from native endianess to foreign,
// call dtNavMeshHeaderSwapEndian() after the data has been swapped.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int dataSize);

#endif // DETOURNAVMESHBUILDER_H
