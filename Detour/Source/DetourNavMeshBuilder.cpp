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
#include "DetourNavMeshBuilder.h"



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

/*
static int queryPolygons(dtMeshHeader* header,
						 const float* qmin, const float* qmax,
						 unsigned short* polys, const int maxPolys)
{
	const dtBVNode* node = &header->bvtree[0];
	const dtBVNode* end = &header->bvtree[header->nbvtree];
	
	// Calculate quantized box
	unsigned short bmin[3], bmax[3];
	// Clamp query box to world box.
	float minx = clamp(qmin[0], header->bmin[0], header->bmax[0]) - header->bmin[0];
	float miny = clamp(qmin[1], header->bmin[1], header->bmax[1]) - header->bmin[1];
	float minz = clamp(qmin[2], header->bmin[2], header->bmax[2]) - header->bmin[2];
	float maxx = clamp(qmax[0], header->bmin[0], header->bmax[0]) - header->bmin[0];
	float maxy = clamp(qmax[1], header->bmin[1], header->bmax[1]) - header->bmin[1];
	float maxz = clamp(qmax[2], header->bmin[2], header->bmax[2]) - header->bmin[2];
	// Quantize
	bmin[0] = (unsigned short)(header->bvquant * minx) & 0xfffe;
	bmin[1] = (unsigned short)(header->bvquant * miny) & 0xfffe;
	bmin[2] = (unsigned short)(header->bvquant * minz) & 0xfffe;
	bmax[0] = (unsigned short)(header->bvquant * maxx + 1) | 1;
	bmax[1] = (unsigned short)(header->bvquant * maxy + 1) | 1;
	bmax[2] = (unsigned short)(header->bvquant * maxz + 1) | 1;
	
	// Traverse tree
	dtPolyRef base = getTileId(tile);
	int n = 0;
	while (node < end)
	{
		bool overlap = checkOverlapBox(bmin, bmax, node->bmin, node->bmax);
		bool isLeafNode = node->i >= 0;
		
		if (isLeafNode && overlap)
		{
			if (n < maxPolys)
				polys[n++] = base | (dtPolyRef)node->i;
		}
		
		if (overlap || isLeafNode)
			node++;
		else
		{
			const int escapeIndex = -node->i;
			node += escapeIndex;
		}
	}
	
	return n;
}

dtMeshHeader* header
{
	bool dtNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest) const
	{
		unsigned int salt, it, ip;
		dtDecodePolyId(ref, salt, it, ip);
		if (it >= (unsigned int)m_maxTiles) return false;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
		const dtMeshHeader* header = m_tiles[it].header;
		
		if (ip >= (unsigned int)header->npolys) return false;
		const dtPoly* poly = &header->polys[ip];
		
		float closestDistSqr = FLT_MAX;
		const dtPolyDetail* pd = &header->dmeshes[ip];
		
		for (int j = 0; j < pd->ntris; ++j)
		{
			const unsigned char* t = &header->dtris[(pd->tbase+j)*4];
			const float* v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < poly->nv)
					v[k] = &header->verts[poly->v[t[k]]*3];
				else
					v[k] = &header->dverts[(pd->vbase+(t[k]-poly->nv))*3];
			}
			float pt[3];
			closestPtPointTriangle(pt, pos, v[0], v[1], v[2]);
			float d = vdistSqr(pos, pt);
			if (d < closestDistSqr)
			{
				vcopy(closest, pt);
				closestDistSqr = d;
			}
		}
		
		return true;
	}
}

unsigned short findNearestPoly(dtMeshHeader* header, const float* center, const float* extents)
{
	// Get nearby polygons from proximity grid.
	float bmin[3], bmax[3];
	bmin[0] = center[0] - extents[0];
	bmin[1] = center[1] - extents[1];
	bmin[2] = center[2] - extents[2];
	bmax[0] = center[0] + extents[0];
	bmax[1] = center[1] + extents[1];
	bmax[2] = center[2] + extents[2];
	unsigned short polys[128];
	int npolys = queryPolygons(header, bmin, bmax, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	unsigned short nearest = 0xffff;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < npolys; ++i)
	{
		dtPolyRef ref = polys[i];
		float closest[3];

		if (!closestPointOnPoly(ref, center, closest))
			continue;
			
		float d = vdistSqr(center, closest);
		if (d < nearestDistanceSqr)
		{
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}
*/
	
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	if (params->nvp > DT_VERTS_PER_POLYGON)
		return false;
	if (params->vertCount >= 0xffff)
		return false;
	if (!params->vertCount || !params->verts)
		return false;
	if (!params->polyCount || !params->polys)
		return false;
	if (!params->detailMeshes || !params->detailVerts || !params->detailTris)
		return false;

	const int nvp = params->nvp;
	
	// Off-mesh connectionss are stored as polygons, adjust values.
	const int totPolyCount = params->polyCount + params->offMeshConCount;
	const int totVertCount = params->vertCount + params->offMeshConCount*2;
	
	// Find portal edges which are at tile borders.
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		const unsigned short* p = &params->polys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			int nj = j+1;
			if (nj >= nvp || p[nj] == 0xffff) nj = 0;
			const unsigned short* va = &params->verts[p[j]*3];
			const unsigned short* vb = &params->verts[p[nj]*3];
			
			edgeCount++;
			
			if (params->tileSize > 0)
			{
				if (va[0] == params->tileSize && vb[0] == params->tileSize)
					portalCount++; // x+
				else if (va[2] == params->tileSize && vb[2] == params->tileSize)
					portalCount++; // z+
				else if (va[0] == 0 && vb[0] == 0)
					portalCount++; // x-
				else if (va[2] == 0 && vb[2] == 0)
					portalCount++; // z-
			}
		}
	}

	const int maxLinkCount = edgeCount + portalCount*2 + params->offMeshConCount*4;
	
	
	// Find unique detail vertices.
	int uniqueDetailVertCount = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		const unsigned short* p = &params->polys[i*nvp*2];
		int ndv = params->detailMeshes[i*4+1];
		int nv = 0;
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			nv++;
		}
		ndv -= nv;
		uniqueDetailVertCount += ndv;
	}
	
	// Calculate data size
	const int headerSize = align4(sizeof(dtMeshHeader));
	const int vertsSize = align4(sizeof(float)*3*totVertCount);
	const int polysSize = align4(sizeof(dtPoly)*totPolyCount);
	const int linksSize = align4(sizeof(dtLink)*maxLinkCount);
	const int detailMeshesSize = align4(sizeof(dtPolyDetail)*params->polyCount);
	const int detailVertsSize = align4(sizeof(float)*3*uniqueDetailVertCount);
	const int detailTrisSize = align4(sizeof(unsigned char)*4*params->detailTriCount);
	const int bvTreeSize = align4(sizeof(dtBVNode)*params->polyCount*2);
	const int offMeshConsSize = align4(sizeof(dtOffMeshConnection)*params->offMeshConCount);
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize +
						 bvTreeSize + offMeshConsSize;
						 
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
	dtBVNode* navBvtree = (dtBVNode*)d; d += bvTreeSize;
	dtOffMeshConnection* offMeshCons = (dtOffMeshConnection*)d; d += offMeshConsSize;
	
	
	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->polyCount = totPolyCount;
	header->vertCount = totVertCount;
	header->maxLinkCount = maxLinkCount;
	vcopy(header->bmin, params->bmin);
	vcopy(header->bmax, params->bmax);
	header->detailMeshCount = params->polyCount;
	header->detailVertCount = uniqueDetailVertCount;
	header->detailTriCount = params->detailTriCount;
	header->bvQuantFactor = 1.0f / params->cs;
	header->offMeshBase = params->polyCount;
	header->walkableHeight = params->walkableHeight;
	header->walkableRadius = params->walkableRadius;
	header->walkableClimb = params->walkableClimb;
	header->offMeshConCount = params->offMeshConCount;
	header->bvNodeCount = params->polyCount*2;
	
	const int offMeshVertsBase = params->vertCount;
	const int offMeshPolyBase = params->polyCount;
	
	// Store vertices
	// Mesh vertices
	for (int i = 0; i < params->vertCount; ++i)
	{
		const unsigned short* iv = &params->verts[i*3];
		float* v = &navVerts[i*3];
		v[0] = params->bmin[0] + iv[0] * params->cs;
		v[1] = params->bmin[1] + iv[1] * params->ch;
		v[2] = params->bmin[2] + iv[2] * params->cs;
	}
	// Off-mesh link vertices.
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		const float* linkv = &params->offMeshConVerts[i*2*3];
		float* v = &navVerts[(offMeshVertsBase + i*2)*3];
		vcopy(&v[0], &linkv[0]);
		vcopy(&v[3], &linkv[3]);
	}
	
	// Store polygons
	// Mesh polys
	const unsigned short* src = params->polys;
	for (int i = 0; i < params->polyCount; ++i)
	{
		dtPoly* p = &navPolys[i];
		p->vertCount = 0;
		p->flags = DT_POLY_GROUND;
		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == 0xffff) break;
			p->verts[j] = src[j];
			p->neis[j] = (src[nvp+j]+1) & 0xffff;
			p->vertCount++;
		}
		src += nvp*2;
	}
	// Off-mesh connection vertices.
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		dtPoly* p = &navPolys[offMeshPolyBase+i];
		p->vertCount = 2;
		p->verts[0] = (unsigned short)(offMeshVertsBase + i*2+0);
		p->verts[1] = (unsigned short)(offMeshVertsBase + i*2+1);
		p->flags = DT_POLY_OFFMESH_CONNECTION; // Off-mesh link poly.
	}
	
	// Store portal edges.
	if (params->tileSize > 0)
	{
		for (int i = 0; i < params->polyCount; ++i)
		{
			dtPoly* poly = &navPolys[i];
			for (int j = 0; j < poly->vertCount; ++j)
			{
				int nj = j+1;
				if (nj >= poly->vertCount) nj = 0;

				const unsigned short* va = &params->verts[poly->verts[j]*3];
				const unsigned short* vb = &params->verts[poly->verts[nj]*3];
							
				if (va[0] == params->tileSize && vb[0] == params->tileSize) // x+
					poly->neis[j] = DT_EXT_LINK | 0;
				else if (va[2] == params->tileSize && vb[2]  == params->tileSize) // z+
					poly->neis[j] = DT_EXT_LINK | 1;
				else if (va[0] == 0 && vb[0] == 0) // x-
					poly->neis[j] = DT_EXT_LINK | 2;
				else if (va[2] == 0 && vb[2] == 0) // z-
					poly->neis[j] = DT_EXT_LINK | 3;
			}
		}
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	unsigned short vbase = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		dtPolyDetail& dtl = navDMeshes[i];
		const int vb = params->detailMeshes[i*4+0];
		const int ndv = params->detailMeshes[i*4+1];
		const int nv = navPolys[i].vertCount;
		dtl.vertBase = vbase;
		dtl.vertCount = ndv-nv;
		dtl.triBase = params->detailMeshes[i*4+2];
		dtl.triCount = params->detailMeshes[i*4+3];
		// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
		if (ndv-nv)
		{
			memcpy(&navDVerts[vbase*3], &params->detailVerts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
			vbase += ndv-nv;
		}
	}
	// Store triangles.
	memcpy(navDTris, params->detailTris, sizeof(unsigned char)*4*params->detailTriCount);

	// Store and create BVtree.
	// TODO: take detail mesh into account! use byte per bbox extent?
	createBVTree(params->verts, params->vertCount, params->polys, params->polyCount,
				 nvp, params->cs, params->ch, params->polyCount*2, navBvtree);
	
	// Store Off-Mesh connections.
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &offMeshCons[i];
		con->poly = offMeshPolyBase + i;
		// Copy connection end-points.
		const float* endPts = &params->offMeshConVerts[i*2*3];
		vcopy(&con->pos[0], &endPts[0]);
		vcopy(&con->pos[3], &endPts[3]);
		con->rad = params->offMeshConRad[i];
		con->flags = params->offMeshConDir[i];
	}
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}
