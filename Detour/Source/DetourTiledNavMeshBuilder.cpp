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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DetourTiledNavMesh.h"

template<class T> inline T abs(T a) { return a < 0 ? -a : a; }
template<class T> inline T min(T a, T b) { return a < b ? a : b; }
template<class T> inline T max(T a, T b) { return a > b ? a : b; }

bool dtCreateNavMeshTileData(const unsigned short* verts, const int nverts,
							 const unsigned short* polys, const int npolys, const int nvp,
							 const float* bmin, const float* bmax, float cs, float ch, int tileSize, int walkableClimb,
							 unsigned char** outData, int* outDataSize)
{
	if (nvp != DT_TILE_VERTS_PER_POLYGON)
		return false;
	if (nverts >= 0xffff)
		return false;
	
	if (!nverts)
		return false;
	if (!npolys)
		return false;
	
/*	int nportals[4] = {0,0,0,0};
	
	// Find portals.
	for (int i = 0; i < npolys; ++i)
	{
		const unsigned short* p = &polys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			int nj = j+1;
			if (nj >= nvp || p[nj] == 0xffff) nj = 0;
			const unsigned short* va = &verts[p[j]*3];
			const unsigned short* vb = &verts[p[nj]*3];

			if (va[0] == tileSize && vb[0] == tileSize)
				nportals[0]++; // x+
			else if (va[2] == tileSize && vb[2]  == tileSize)
				nportals[1]++; // z+
			else if (va[0] == 0 && vb[0] == 0)
				nportals[2]++; // x-
			else if (va[2] == 0 && vb[2] == 0)
				nportals[3]++; // z-
		}
	}
*/

	// Find portals.
	int nedges = 0;
	int nportals = 0;
	for (int i = 0; i < npolys; ++i)
	{
		const unsigned short* p = &polys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			int nj = j+1;
			if (nj >= nvp || p[nj] == 0xffff) nj = 0;
			const unsigned short* va = &verts[p[j]*3];
			const unsigned short* vb = &verts[p[nj]*3];
			
			nedges++;
			
			if (va[0] == tileSize && vb[0] == tileSize)
				nportals++; // x+
			else if (va[2] == tileSize && vb[2]  == tileSize)
				nportals++; // z+
			else if (va[0] == 0 && vb[0] == 0)
				nportals++; // x-
			else if (va[2] == 0 && vb[2] == 0)
				nportals++; // z-
		}
	}
	
	// Calculate data size
	const int headerSize = sizeof(dtTileHeader);
	const int vertsSize = sizeof(float)*3*nverts;
	const int polysSize = sizeof(dtTilePoly)*npolys;
//	const int portalsSize = sizeof(dtTilePortal)*(nportals[0]+nportals[1]+nportals[2]+nportals[3]);
//	const int dataSize = headerSize + vertsSize + polysSize + portalsSize;
	const int linksSize = sizeof(dtTileLink)*(nedges + nportals*2);
	const int dataSize = headerSize + vertsSize + polysSize + linksSize;
	unsigned char* data = new unsigned char[dataSize];
	if (!data)
		return false;
	memset(data, 0, dataSize);
	
	dtTileHeader* header = (dtTileHeader*)(data);
	float* navVerts = (float*)(data + headerSize);
	dtTilePoly* navPolys = (dtTilePoly*)(data + headerSize + vertsSize);
/*	dtTilePortal* portals[4];
	portals[0] = (dtTilePortal*)(data + headerSize + vertsSize + polysSize);
	portals[1] = portals[0] + nportals[0];
	portals[2] = portals[1] + nportals[1];
	portals[3] = portals[2] + nportals[2];*/
	
	// Store header
	header->magic = DT_TILE_NAVMESH_MAGIC;
	header->version = DT_TILE_NAVMESH_VERSION;
	header->npolys = npolys;
	header->nverts = nverts;
	header->maxlinks = nedges + nportals*2;
/*	header->nportals[0] = nportals[0];
	header->nportals[1] = nportals[1];
	header->nportals[2] = nportals[2];
	header->nportals[3] = nportals[3];*/
	header->cs = cs;
	header->bmin[0] = bmin[0];
	header->bmin[1] = bmin[1];
	header->bmin[2] = bmin[2];
	header->bmax[0] = bmax[0];
	header->bmax[1] = bmax[1];
	header->bmax[2] = bmax[2];
	
	// Store vertices
	for (int i = 0; i < nverts; ++i)
	{
		const unsigned short* iv = &verts[i*3];
		float* v = &navVerts[i*3];
		v[0] = bmin[0] + iv[0] * cs;
		v[1] = bmin[1] + iv[1] * ch;
		v[2] = bmin[2] + iv[2] * cs;
	}
	
	// Store polygons
	const unsigned short* src = polys;
	for (int i = 0; i < npolys; ++i)
	{
		dtTilePoly* p = &navPolys[i];
		p->nv = 0;
		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == 0xffff) break;
			p->v[j] = src[j];
			p->n[j] = (src[nvp+j]+1) & 0xffff;
			p->nv++;
		}
		src += nvp*2;
	}

	// Store portals.
	for (int i = 0; i < npolys; ++i)
	{
		dtTilePoly* poly = &navPolys[i];
		for (int j = 0; j < poly->nv; ++j)
		{
			int nj = j+1;
			if (nj >= poly->nv) nj = 0;

			const unsigned short* va = &verts[poly->v[j]*3];
			const unsigned short* vb = &verts[poly->v[nj]*3];
						
			if (va[0] == tileSize && vb[0] == tileSize) // x+
				poly->n[j] = 0x8000 | 0;
			else if (va[2] == tileSize && vb[2]  == tileSize) // z+
				poly->n[j] = 0x8000 | 1;
			else if (va[0] == 0 && vb[0] == 0) // x-
				poly->n[j] = 0x8000 | 2;
			else if (va[2] == 0 && vb[2] == 0) // z-
				poly->n[j] = 0x8000 | 3;
		}
	}
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}
