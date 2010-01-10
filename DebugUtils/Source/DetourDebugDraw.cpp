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

#include "DebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"


static float distancePtLine2d(const float* pt, const float* p, const float* q)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d != 0) t /= d;
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}

static void drawPolyBoundaries(duDebugDraw* dd, const dtMeshHeader* header,
							   const unsigned int col, const float linew,
							   bool inner)
{
	static const float thr = 0.01f*0.01f;

	dd->begin(DU_DRAW_LINES, linew);

	for (int i = 0; i < header->npolys; ++i)
	{
		const dtPoly* p = &header->polys[i];
		
		if (p->flags & DT_POLY_OFFMESH_LINK) continue;
		
		const dtPolyDetail* pd = &header->dmeshes[i];
		
		for (int j = 0, nj = (int)p->nv; j < nj; ++j)
		{
			unsigned int c = col;
			if (inner)
			{
				if (p->n[j] == 0) continue;
				if (p->n[j] & 0x8000)
				{
					bool con = false;
					for (int k = 0; k < p->nlinks; ++k)
					{
						if (header->links[p->links+k].e == j)
						{
							con = true;
							break;
						}
					}
					if (con)
						c = duRGBA(255,255,255,64);
					else
						c = duRGBA(0,0,0,128);
				}
				else
					c = duRGBA(0,48,64,32);
			}
			else
			{
				if (p->n[j] != 0) continue;
			}
			
			const float* v0 = &header->verts[p->v[j]*3];
			const float* v1 = &header->verts[p->v[(j+1)%nj]*3];
			
			// Draw detail mesh edges which align with the actual poly edge.
			// This is really slow.
			for (int k = 0; k < pd->ntris; ++k)
			{
				const unsigned char* t = &header->dtris[(pd->tbase+k)*4];
				const float* tv[3];
				for (int m = 0; m < 3; ++m)
				{
					if (t[m] < p->nv)
						tv[m] = &header->verts[p->v[t[m]]*3];
					else
						tv[m] = &header->dverts[(pd->vbase+(t[m]-p->nv))*3];
				}
				for (int m = 0, n = 2; m < 3; n=m++)
				{
					if (((t[3] >> (n*2)) & 0x3) == 0) continue;	// Skip inner detail edges.
					if (distancePtLine2d(tv[n],v0,v1) < thr &&
						distancePtLine2d(tv[m],v0,v1) < thr)
					{
						dd->vertex(tv[n], c);
						dd->vertex(tv[m], c);
					}
				}
			}
		}
	}
	dd->end();
}

static void drawMeshTile(duDebugDraw* dd, const dtNavMesh* mesh, const dtMeshTile* tile, bool drawClosedList)
{
	const dtMeshHeader* header = tile->header;
	dtPolyRef base = mesh->getTileId(tile);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < header->npolys; ++i)
	{
		const dtPoly* p = &header->polys[i];
		if (p->flags & DT_POLY_OFFMESH_LINK)	// Skip off-mesh links.
			continue;
			
		const dtPolyDetail* pd = &header->dmeshes[i];

		unsigned int col;
		if (drawClosedList && mesh->isInClosedList(base | (dtPolyRef)i))
			col = duRGBA(255,196,0,64);
		else
			col = duRGBA(0,196,255,64);
		
		for (int j = 0; j < pd->ntris; ++j)
		{
			const unsigned char* t = &header->dtris[(pd->tbase+j)*4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->nv)
					dd->vertex(&header->verts[p->v[t[k]]*3], col);
				else
					dd->vertex(&header->dverts[(pd->vbase+t[k]-p->nv)*3], col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < header->npolys; ++i)
	{
		const dtPoly* p = &header->polys[i];
		if ((p->flags & DT_POLY_OFFMESH_LINK) == 0)	// Skip regular polys.
			continue;
			
		unsigned int col;
		if (drawClosedList && mesh->isInClosedList(base | (dtPolyRef)i))
			col = duRGBA(255,196,0,220);
		else
			col = duRGBA(0,196,255,220);
		
		const float* va = &header->verts[p->v[0]*3];
		const float* vb = &header->verts[p->v[1]*3];
		
		duDebugDrawArc(dd, va[0],va[1]+0.1f,va[2], vb[0],vb[1]+0.1f,vb[2], 0.25f, col, 2.0f);
	}
	dd->end();
	
	// Draw inter poly boundaries
	drawPolyBoundaries(dd, header, duRGBA(0,48,64,32), 1.5f, true);
	
	// Draw outer poly boundaries
	drawPolyBoundaries(dd, header, duRGBA(0,48,64,220), 2.5f, false);

	const unsigned int vcol = duRGBA(0,0,0,196);
	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < header->nverts; ++i)
	{
		const float* v = &header->verts[i*3];
		dd->vertex(v[0], v[1], v[2], vcol);
	}
	dd->end();
	
	// Draw portals
	/*	glBegin(GL_LINES);
	 
	 for (int i = 0; i < header->nportals[0]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[0][i];		
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmin[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmin[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmin[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmax[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmax[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmax[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmax[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmin[0]);
	 }
	 for (int i = 0; i < header->nportals[1]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[1][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmax[2]-0.1f);
	 }
	 for (int i = 0; i < header->nportals[2]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[2][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmin[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmin[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmin[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmax[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmax[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmax[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmax[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmin[0]);
	 }
	 for (int i = 0; i < header->nportals[3]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[3][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmin[2]+0.1f);
	 }
	 glEnd();*/
}

void duDebugDrawNavMesh(duDebugDraw* dd, const dtNavMesh* mesh, bool drawClosedList)
{
	if (!mesh) return;
	
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile->header) continue;
		drawMeshTile(dd, mesh, tile, drawClosedList);
	}
}


static void drawMeshTileBVTree(duDebugDraw* dd, const dtNavMesh* mesh, const dtMeshTile* tile)
{
	const dtMeshHeader* header = tile->header;
	
	// Draw BV nodes.
	const float cs = 1.0f / header->bvquant;
	dd->begin(DU_DRAW_LINES, 1.0f);
	for (int i = 0; i < header->nbvtree; ++i)
	{
		const dtBVNode* n = &header->bvtree[i];
		if (n->i < 0) // Leaf indices are positive.
			continue;
		duAppendBoxWire(dd, header->bmin[0] + n->bmin[0]*cs,
						header->bmin[1] + n->bmin[1]*cs,
						header->bmin[2] + n->bmin[2]*cs,
						header->bmin[0] + n->bmax[0]*cs,
						header->bmin[1] + n->bmax[1]*cs,
						header->bmin[2] + n->bmax[2]*cs,
						duRGBA(255,255,255,128));
	}
	dd->end();
	
	// Draw portals
	/*	glBegin(GL_LINES);
	 
	 for (int i = 0; i < header->nportals[0]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[0][i];		
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmin[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmin[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmin[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmax[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmax[1], p->bmax[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmax[0]);
	 
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmax[0]);
	 glVertex3f(header->bmax[0]-0.1f, p->bmin[1], p->bmin[0]);
	 }
	 for (int i = 0; i < header->nportals[1]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[1][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmax[2]-0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmax[2]-0.1f);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmax[2]-0.1f);
	 }
	 for (int i = 0; i < header->nportals[2]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[2][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmin[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmin[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmin[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmax[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmax[1], p->bmax[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmax[0]);
	 
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmax[0]);
	 glVertex3f(header->bmin[0]+0.1f, p->bmin[1], p->bmin[0]);
	 }
	 for (int i = 0; i < header->nportals[3]; ++i)
	 {
	 const dtTilePortal* p = &header->portals[3][i];
	 if (p->ncon)
	 glColor4ub(255,255,255,192);
	 else
	 glColor4ub(255,0,0,64);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmin[0], p->bmax[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmax[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmin[2]+0.1f);
	 
	 glVertex3f(p->bmax[0], p->bmin[1], header->bmin[2]+0.1f);
	 glVertex3f(p->bmin[0], p->bmin[1], header->bmin[2]+0.1f);
	 }
	 glEnd();*/
}

void duDebugDrawNavMeshBVTree(duDebugDraw* dd, const dtNavMesh* mesh)
{
	if (!mesh) return;
	
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile->header) continue;
		drawMeshTileBVTree(dd, mesh, tile);
	}
}

void duDebugDrawNavMeshPoly(duDebugDraw* dd, const dtNavMesh* mesh, dtPolyRef ref, const unsigned int col)
{
	int ip = 0;
	const dtMeshTile* tile = mesh->getTileByRef(ref, &ip);
	if (!tile)
		return;
	const dtMeshHeader* header = tile->header;
	const dtPoly* p = &header->polys[ip];
	
	const unsigned int c = (col & 0x00ffffff) | (64 << 24);
	
	if (p->flags & DT_POLY_OFFMESH_LINK)
	{
		const float* va = &header->verts[p->v[0]*3];
		const float* vb = &header->verts[p->v[1]*3];
		duDebugDrawArc(dd, va[0],va[1]+0.1f,va[2], vb[0],vb[1]+0.1f,vb[2], 0.25f, c, 2.0f);
	}
	else
	{
		const dtPolyDetail* pd = &header->dmeshes[ip];

		dd->begin(DU_DRAW_TRIS);
		for (int i = 0; i < pd->ntris; ++i)
		{
			const unsigned char* t = &header->dtris[(pd->tbase+i)*4];
			for (int j = 0; j < 3; ++j)
			{
				if (t[j] < p->nv)
					dd->vertex(&header->verts[p->v[t[j]]*3], c);
				else
					dd->vertex(&header->dverts[(pd->vbase+t[j]-p->nv)*3], c);
			}
		}
		dd->end();
	}
}

