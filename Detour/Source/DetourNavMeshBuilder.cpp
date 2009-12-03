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
#include "DetourNavMesh.h"
#include "DetourCommon.h"



struct BVItem
{
	unsigned short bmin[3];
	unsigned short bmax[3];
	int i;
};

static int compareItemX(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[0] < b->bmin[0])
		return -1;
	if (a->bmin[0] > b->bmin[0])
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[1] < b->bmin[1])
		return -1;
	if (a->bmin[1] > b->bmin[1])
		return 1;
	return 0;
}

static int compareItemZ(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[2] < b->bmin[2])
		return -1;
	if (a->bmin[2] > b->bmin[2])
		return 1;
	return 0;
}

static void calcExtends(BVItem* items, int nitems, int imin, int imax,
						unsigned short* bmin, unsigned short* bmax)
{
	bmin[0] = items[imin].bmin[0];
	bmin[1] = items[imin].bmin[1];
	bmin[2] = items[imin].bmin[2];
	
	bmax[0] = items[imin].bmax[0];
	bmax[1] = items[imin].bmax[1];
	bmax[2] = items[imin].bmax[2];
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BVItem& it = items[i];
		if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
		if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
		if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];
		
		if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
		if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
		if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
	}
}

inline int longestAxis(unsigned short x, unsigned short y, unsigned short z)
{
	int	axis = 0;
	unsigned short maxVal = x;
	if (y > maxVal)
	{
		axis = 1;
		maxVal = y;
	}
	if (z > maxVal)
	{
		axis = 2;
		maxVal = z;
	}
	return axis;
}

static void subdivide(BVItem* items, int nitems, int imin, int imax, int& curNode, dtBVNode* nodes)
{
	int inum = imax - imin;
	int icur = curNode;
	
	dtBVNode& node = nodes[curNode++];
	
	if (inum == 1)
	{
		// Leaf
		node.bmin[0] = items[imin].bmin[0];
		node.bmin[1] = items[imin].bmin[1];
		node.bmin[2] = items[imin].bmin[2];
		
		node.bmax[0] = items[imin].bmax[0];
		node.bmax[1] = items[imin].bmax[1];
		node.bmax[2] = items[imin].bmax[2];
		
		node.i = items[imin].i;
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax[0] - node.bmin[0],
							   node.bmax[1] - node.bmin[1],
							   node.bmax[2] - node.bmin[2]);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemY);
		}
		else
		{
			// Sort along z-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemZ);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, curNode, nodes);
		// Right
		subdivide(items, nitems, isplit, imax, curNode, nodes);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

static int createBVTree(const unsigned short* verts, const int nverts,
						const unsigned short* polys, const int npolys, const int nvp,
						float cs, float ch,
						int nnodes, dtBVNode* nodes)
{
	// Build tree
	BVItem* items = new BVItem[npolys];
	for (int i = 0; i < npolys; i++)
	{
		BVItem& it = items[i];
		it.i = i;
		// Calc polygon bounds.
		const unsigned short* p = &polys[i*nvp*2];
		it.bmin[0] = it.bmax[0] = verts[p[0]*3+0];
		it.bmin[1] = it.bmax[1] = verts[p[0]*3+1];
		it.bmin[2] = it.bmax[2] = verts[p[0]*3+2];
		
		for (int j = 1; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			unsigned short x = verts[p[j]*3+0];
			unsigned short y = verts[p[j]*3+1];
			unsigned short z = verts[p[j]*3+2];
			
			if (x < it.bmin[0]) it.bmin[0] = x;
			if (y < it.bmin[1]) it.bmin[1] = y;
			if (z < it.bmin[2]) it.bmin[2] = z;
			
			if (x > it.bmax[0]) it.bmax[0] = x;
			if (y > it.bmax[1]) it.bmax[1] = y;
			if (z > it.bmax[2]) it.bmax[2] = z;
		}
		// Remap y
		it.bmin[1] = (unsigned short)floorf((float)it.bmin[1]*ch/cs);
		it.bmax[1] = (unsigned short)ceilf((float)it.bmax[1]*ch/cs);
	}
	
	int curNode = 0;
	subdivide(items, npolys, 0, npolys, curNode, nodes);
	
	delete [] items;
	
	return curNode;
}


bool dtCreateNavMeshData(const unsigned short* verts, const int nverts,
						 const unsigned short* polys, const int npolys, const int nvp,
						 const unsigned short* dmeshes, const float* dverts, const int ndverts,
						 const unsigned char* dtris, const int ndtris, 
						 const float* bmin, const float* bmax, float cs, float ch, int tileSize, int walkableClimb,
						 unsigned char** outData, int* outDataSize)
{
	if (nvp != DT_VERTS_PER_POLYGON)
		return false;
	if (nverts >= 0xffff)
		return false;
	
	if (!nverts)
		return false;
	if (!npolys)
		return false;
	if (!dmeshes || !dverts || ! dtris)
		return false;
	
	// Find portal edges which are at tile borders.
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
			
			if (tileSize > 0)
			{
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
	}

	const int maxLinks = nedges + nportals*2;
	
	
	// Find unique detail vertices.
	int uniqueDetailVerts = 0;
	if (dmeshes)
	{
		for (int i = 0; i < npolys; ++i)
		{
			const unsigned short* p = &polys[i*nvp*2];
			int ndv = dmeshes[i*4+1];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == 0xffff) break;
				nv++;
			}
			ndv -= nv;
			uniqueDetailVerts += ndv;
		}
	}
	
	// Calculate data size
	const int headerSize = align4(sizeof(dtMeshHeader));
	const int vertsSize = align4(sizeof(float)*3*nverts);
	const int polysSize = align4(sizeof(dtPoly)*npolys);
	const int linksSize = align4(sizeof(dtLink)*maxLinks);
	const int detailMeshesSize = align4(sizeof(dtPolyDetail)*npolys);
	const int detailVertsSize = align4(sizeof(float)*3*uniqueDetailVerts);
	const int detailTrisSize = align4(sizeof(unsigned char)*4*ndtris);
	const int bvtreeSize = align4(sizeof(dtBVNode)*npolys*2);
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize + bvtreeSize;
	unsigned char* data = new unsigned char[dataSize];
	if (!data)
		return false;
	memset(data, 0, dataSize);
	
	unsigned char* d = data;
	dtMeshHeader* header = (dtMeshHeader*)d; d += headerSize;
	float* navVerts = (float*)d; d += vertsSize;
	dtPoly* navPolys = (dtPoly*)d; d += polysSize;
	d += linksSize;
	dtPolyDetail* navDMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	float* navDVerts = (float*)d; d += detailVertsSize;
	unsigned char* navDTris = (unsigned char*)d; d += detailTrisSize;
	dtBVNode* navBvtree = (dtBVNode*)d; d += bvtreeSize;
	
	
	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->npolys = npolys;
	header->nverts = nverts;
	header->maxlinks = maxLinks;
	header->bmin[0] = bmin[0];
	header->bmin[1] = bmin[1];
	header->bmin[2] = bmin[2];
	header->bmax[0] = bmax[0];
	header->bmax[1] = bmax[1];
	header->bmax[2] = bmax[2];
	header->ndmeshes = npolys;
	header->ndverts = uniqueDetailVerts;
	header->ndtris = ndtris;
	header->bvquant = 1.0f/cs;
	
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
		dtPoly* p = &navPolys[i];
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

	// Store portal edges.
	if (tileSize > 0)
	{
		for (int i = 0; i < npolys; ++i)
		{
			dtPoly* poly = &navPolys[i];
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
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	unsigned short vbase = 0;
	for (int i = 0; i < npolys; ++i)
	{
		dtPolyDetail& dtl = navDMeshes[i];
		const int vb = dmeshes[i*4+0];
		const int ndv = dmeshes[i*4+1];
		const int nv = navPolys[i].nv;
		dtl.vbase = vbase;
		dtl.nverts = ndv-nv;
		dtl.tbase = dmeshes[i*4+2];
		dtl.ntris = dmeshes[i*4+3];
		// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
		if (ndv-nv)
		{
			memcpy(&navDVerts[vbase*3], &dverts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
			vbase += ndv-nv;
		}
	}
	// Store triangles.
	memcpy(navDTris, dtris, sizeof(unsigned char)*4*ndtris);

	// Store and create BVtree.
	// TODO: take detail mesh into account! use byte per bbox extent?
	header->nbvtree = createBVTree(verts, nverts, polys, npolys, nvp,
								   cs, ch, npolys*2, navBvtree);
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}
