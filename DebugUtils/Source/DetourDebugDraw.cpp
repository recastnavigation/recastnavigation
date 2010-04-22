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

#include <math.h>
#include "DebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"


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

static void drawPolyBoundaries(duDebugDraw* dd, const dtMeshTile* tile,
							   const unsigned int col, const float linew,
							   bool inner)
{
	static const float thr = 0.01f*0.01f;

	dd->begin(DU_DRAW_LINES, linew);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		
		if (p->type == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
		
		const dtPolyDetail* pd = &tile->detailMeshes[i];
		
		for (int j = 0, nj = (int)p->vertCount; j < nj; ++j)
		{
			unsigned int c = col;
			if (inner)
			{
				if (p->neis[j] == 0) continue;
				if (p->neis[j] & DT_EXT_LINK)
				{
					bool con = false;
					for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
					{
						if (tile->links[k].edge == j)
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
				if (p->neis[j] != 0) continue;
			}
			
			const float* v0 = &tile->verts[p->verts[j]*3];
			const float* v1 = &tile->verts[p->verts[(j+1) % nj]*3];
			
			// Draw detail mesh edges which align with the actual poly edge.
			// This is really slow.
			for (int k = 0; k < pd->triCount; ++k)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase+k)*4];
				const float* tv[3];
				for (int m = 0; m < 3; ++m)
				{
					if (t[m] < p->vertCount)
						tv[m] = &tile->verts[p->verts[t[m]]*3];
					else
						tv[m] = &tile->detailVerts[(pd->vertBase+(t[m]-p->vertCount))*3];
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

static void drawMeshTile(duDebugDraw* dd, const dtNavMesh& mesh, const dtMeshTile* tile, unsigned char flags)
{
	dtPolyRef base = mesh.getTilePolyRefBase(tile);

	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->type == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
			continue;
			
		const dtPolyDetail* pd = &tile->detailMeshes[i];

		unsigned int col;
		if ((flags & DU_DRAWNAVMESH_CLOSEDLIST) && mesh.isInClosedList(base | (dtPolyRef)i))
			col = duRGBA(255,196,0,64);
		else
		{
			if (p->area == 0) // Treat zero area type as default.
				col = duRGBA(0,192,255,64);
			else
				col = duIntToCol(p->area, 64);
		}
		
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->vertCount)
					dd->vertex(&tile->verts[p->verts[t[k]]*3], col);
				else
					dd->vertex(&tile->detailVerts[(pd->vertBase+t[k]-p->vertCount)*3], col);
			}
		}
	}
	dd->end();
	
	// Draw inter poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0,48,64,32), 1.5f, true);
	
	// Draw outer poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0,48,64,220), 2.5f, false);

	if (flags & DU_DRAWNAVMESH_OFFMESHCONS)
	{
		dd->begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];
			if (p->type != DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip regular polys.
				continue;
			
			unsigned int col;
			if ((flags & DU_DRAWNAVMESH_CLOSEDLIST) && mesh.isInClosedList(base | (dtPolyRef)i))
				col = duRGBA(255,196,0,220);
			else
				col = duDarkenColor(duIntToCol(p->area, 220));
			
			const dtOffMeshConnection* con = &tile->offMeshCons[i - tile->header->offMeshBase];
			const float* va = &tile->verts[p->verts[0]*3];
			const float* vb = &tile->verts[p->verts[1]*3];

			// Check to see if start and end end-points have links.
			bool startSet = false;
			bool endSet = false;
			for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				if (tile->links[k].edge == 0)
					startSet = true;
				if (tile->links[k].edge == 1)
					endSet = true;
			}
			
			// End points and their on-mesh locations. 
			if (startSet)
			{
				dd->vertex(va[0],va[1],va[2], col);
				dd->vertex(con->pos[0],con->pos[1],con->pos[2], col);
				duAppendCircle(dd, con->pos[0],con->pos[1]+0.1f,con->pos[2], con->rad, duRGBA(0,48,64,196));
			}
			if (endSet)
			{
				dd->vertex(vb[0],vb[1],vb[2], col);
				dd->vertex(con->pos[3],con->pos[4],con->pos[5], col);
				duAppendCircle(dd, con->pos[3],con->pos[4]+0.1f,con->pos[5], con->rad, duRGBA(0,48,64,196));
			}	
			
			// End point vertices.
			dd->vertex(con->pos[0],con->pos[1],con->pos[2], duRGBA(0,48,64,196));
			dd->vertex(con->pos[0],con->pos[1]+0.2f,con->pos[2], duRGBA(0,48,64,196));
			
			dd->vertex(con->pos[3],con->pos[4],con->pos[5], duRGBA(0,48,64,196));
			dd->vertex(con->pos[3],con->pos[4]+0.2f,con->pos[5], duRGBA(0,48,64,196));
			
			// Connection arc.
			duAppendArc(dd, con->pos[0],con->pos[1],con->pos[2], con->pos[3],con->pos[4],con->pos[5], 0.25f,
						(con->flags & 1) ? 0.6f : 0, 0.6f, col);
		}
		dd->end();
	}
	
	
	const unsigned int vcol = duRGBA(0,0,0,196);
	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < tile->header->vertCount; ++i)
	{
		const float* v = &tile->verts[i*3];
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
	 
	dd->depthMask(true);

}

void duDebugDrawNavMesh(duDebugDraw* dd, const dtNavMesh& mesh, unsigned char flags)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTile(dd, mesh, tile, flags);
	}
}


static void drawMeshTileBVTree(duDebugDraw* dd, const dtMeshTile* tile)
{
	// Draw BV nodes.
	const float cs = 1.0f / tile->header->bvQuantFactor;
	dd->begin(DU_DRAW_LINES, 1.0f);
	for (int i = 0; i < tile->header->bvNodeCount; ++i)
	{
		const dtBVNode* n = &tile->bvTree[i];
		if (n->i < 0) // Leaf indices are positive.
			continue;
		duAppendBoxWire(dd, tile->header->bmin[0] + n->bmin[0]*cs,
						tile->header->bmin[1] + n->bmin[1]*cs,
						tile->header->bmin[2] + n->bmin[2]*cs,
						tile->header->bmin[0] + n->bmax[0]*cs,
						tile->header->bmin[1] + n->bmax[1]*cs,
						tile->header->bmin[2] + n->bmax[2]*cs,
						duRGBA(255,255,255,128));
	}
	dd->end();
}

void duDebugDrawNavMeshBVTree(duDebugDraw* dd, const dtNavMesh& mesh)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTileBVTree(dd, tile);
	}
}

/*
static void calcRect(const float* va, const float* vb,
					 float* bmin, float* bmax,
					 int side, float padx, float pady)
{
	if (side == 0 || side == 4)
	{
		bmin[0] = dtMin(va[2],vb[2]) + padx;
		bmin[1] = dtMin(va[1],vb[1]) - pady;
		bmax[0] = dtMax(va[2],vb[2]) - padx;
		bmax[1] = dtMax(va[1],vb[1]) + pady;
	}
	else if (side == 2 || side == 6)
	{
		bmin[0] = dtMin(va[0],vb[0]) + padx;
		bmin[1] = dtMin(va[1],vb[1]) - pady;
		bmax[0] = dtMax(va[0],vb[0]) - padx;
		bmax[1] = dtMax(va[1],vb[1]) + pady;
	}
}
*/

static void drawMeshTilePortal(duDebugDraw* dd, const dtMeshTile* tile)
{
	// Draw portals
	const float padx = 0.01f;
	const float pady = tile->header->walkableClimb;

	dd->begin(DU_DRAW_LINES, 2.0f);

	for (int side = 0; side < 8; ++side)
	{
		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			dtPoly* poly = &tile->polys[i];
			
			// Create new links.
			const int nv = poly->vertCount;
			for (int j = 0; j < nv; ++j)
			{
				// Skip edges which do not point to the right side.
				if (poly->neis[j] != m)
					continue;
				
				// Create new links
				const float* va = &tile->verts[poly->verts[j]*3];
				const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
				
				if (side == 0 || side == 4)
				{
					unsigned int col = side == 0 ? duRGBA(128,0,0,128) : duRGBA(128,0,128,128);

					const float x = va[0] + ((side == 0) ? -0.02f : 0.02f);
					
					dd->vertex(x,va[1]-pady,va[2], col);
					dd->vertex(x,va[1]+pady,va[2], col);

					dd->vertex(x,va[1]+pady,va[2], col);
					dd->vertex(x,vb[1]+pady,vb[2], col);

					dd->vertex(x,vb[1]+pady,vb[2], col);
					dd->vertex(x,vb[1]-pady,vb[2], col);

					dd->vertex(x,vb[1]-pady,vb[2], col);
					dd->vertex(x,va[1]-pady,va[2], col);
					
/*					const float zmin = dtMin(va[2], vb[2]) - padx;
					const float zmax = dtMax(va[2], vb[2]) + padx;
					const float ymin = dtMin(va[1], vb[1]) - pady;
					const float ymax = dtMax(va[1], vb[1]) + pady;
					const float x = va[0] + ((side == 0) ? -0.02f : 0.02f);
					
					dd->vertex(x,ymin,zmin, col);
					dd->vertex(x,ymin,zmax, col);

					dd->vertex(x,ymin,zmax, col);
					dd->vertex(x,ymax,zmax, col);

					dd->vertex(x,ymax,zmax, col);
					dd->vertex(x,ymax,zmin, col);

					dd->vertex(x,ymax,zmin, col);
					dd->vertex(x,ymin,zmin, col);*/
				}
				else if (side == 2 || side == 6)
				{
					unsigned int col = side == 2 ? duRGBA(0,128,0,128) : duRGBA(0,128,128,128);

					const float z = va[2] + ((side == 2) ? -0.02f : 0.02f);
					
					dd->vertex(va[0],va[1]-pady,z, col);
					dd->vertex(va[0],va[1]+pady,z, col);
					
					dd->vertex(va[0],va[1]+pady,z, col);
					dd->vertex(vb[0],vb[1]+pady,z, col);
					
					dd->vertex(vb[0],vb[1]+pady,z, col);
					dd->vertex(vb[0],vb[1]-pady,z, col);
					
					dd->vertex(vb[0],vb[1]-pady,z, col);
					dd->vertex(va[0],va[1]-pady,z, col);
					
					
/*					const float xmin = dtMin(va[0], vb[0]) - padx;
					const float xmax = dtMax(va[0], vb[0]) + padx;
					const float ymin = dtMin(va[1], vb[1]) - pady;
					const float ymax = dtMax(va[1], vb[1]) + pady;
					const float z = va[2] + ((side == 2) ? -0.02f : 0.02f);

					dd->vertex(xmin,ymin,z, col);
					dd->vertex(xmax,ymin,z, col);
					
					dd->vertex(xmax,ymin,z, col);
					dd->vertex(xmax,ymax,z, col);
					
					dd->vertex(xmax,ymax,z, col);
					dd->vertex(xmin,ymax,z, col);
					
					dd->vertex(xmin,ymax,z, col);
					dd->vertex(xmin,ymin,z, col);*/
				}

			}
		}
	}
	
	dd->end();
}

void duDebugDrawNavMeshPortals(duDebugDraw* dd, const dtNavMesh& mesh)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTilePortal(dd, tile);
	}
}

void duDebugDrawNavMeshPoly(duDebugDraw* dd, const dtNavMesh& mesh, dtPolyRef ref, const unsigned int col)
{
	if (!dd) return;
	
	int ip = 0;
	const dtMeshTile* tile = mesh.getTileByPolyRef(ref, &ip);
	if (!tile)
		return;
	const dtPoly* p = &tile->polys[ip];
	
	dd->depthMask(false);
	
	const unsigned int c = (col & 0x00ffffff) | (64 << 24);
	
	if (p->type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		dtOffMeshConnection* con = &tile->offMeshCons[ip - tile->header->offMeshBase];

		dd->begin(DU_DRAW_LINES, 2.0f);

		// Connection arc.
		duAppendArc(dd, con->pos[0],con->pos[1],con->pos[2], con->pos[3],con->pos[4],con->pos[5], 0.25f,
					(con->flags & 1) ? 0.6f : 0, 0.6f, c);
		
		dd->end();

	}
	else
	{
		const dtPolyDetail* pd = &tile->detailMeshes[ip];

		dd->begin(DU_DRAW_TRIS);
		for (int i = 0; i < pd->triCount; ++i)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+i)*4];
			for (int j = 0; j < 3; ++j)
			{
				if (t[j] < p->vertCount)
					dd->vertex(&tile->verts[p->verts[t[j]]*3], c);
				else
					dd->vertex(&tile->detailVerts[(pd->vertBase+t[j]-p->vertCount)*3], c);
			}
		}
		dd->end();
	}
	
	dd->depthMask(true);

}

