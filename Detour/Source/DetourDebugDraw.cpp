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

#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "SDL.h"
#include "SDL_opengl.h"

static void drawBoxWire(float minx, float miny, float minz, float maxx, float maxy, float maxz, const float* col)
{
	glColor4fv(col);
	
	// Top
	glVertex3f(minx, miny, minz);
	glVertex3f(maxx, miny, minz);
	glVertex3f(maxx, miny, minz);
	glVertex3f(maxx, miny, maxz);
	glVertex3f(maxx, miny, maxz);
	glVertex3f(minx, miny, maxz);
	glVertex3f(minx, miny, maxz);
	glVertex3f(minx, miny, minz);
	
	// bottom
	glVertex3f(minx, maxy, minz);
	glVertex3f(maxx, maxy, minz);
	glVertex3f(maxx, maxy, minz);
	glVertex3f(maxx, maxy, maxz);
	glVertex3f(maxx, maxy, maxz);
	glVertex3f(minx, maxy, maxz);
	glVertex3f(minx, maxy, maxz);
	glVertex3f(minx, maxy, minz);
	
	// Sides
	glVertex3f(minx, miny, minz);
	glVertex3f(minx, maxy, minz);
	glVertex3f(maxx, miny, minz);
	glVertex3f(maxx, maxy, minz);
	glVertex3f(maxx, miny, maxz);
	glVertex3f(maxx, maxy, maxz);
	glVertex3f(minx, miny, maxz);
	glVertex3f(minx, maxy, maxz);
}

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

static void drawPolyBoundaries(const dtMeshHeader* header, bool inner)
{
	static const float thr = 0.01f*0.01f;
	
	glBegin(GL_LINES);
	for (int i = 0; i < header->npolys; ++i)
	{
		const dtPoly* p = &header->polys[i];
		const dtPolyDetail* pd = &header->dmeshes[i];
		
		for (int j = 0, nj = (int)p->nv; j < nj; ++j)
		{
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
						glColor4ub(255,255,255,128);
					else
						glColor4ub(0,0,0,128);
				}
				else
					glColor4ub(0,48,64,32);
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
						glVertex3fv(tv[n]);
						glVertex3fv(tv[m]);
					}
				}
			}
		}
	}
	glEnd();
}

static void drawMeshTile(const dtNavMesh* mesh, const dtMeshTile* tile, bool drawClosedList)
{
	const dtMeshHeader* header = tile->header;
	dtPolyRef base = mesh->getTileId(tile);

	glBegin(GL_TRIANGLES);
	for (int i = 0; i < header->npolys; ++i)
	{
		const dtPoly* p = &header->polys[i];
		const dtPolyDetail* pd = &header->dmeshes[i];
		
		if (drawClosedList && mesh->isInClosedList(base | (dtPolyRef)i))
			glColor4ub(255,196,0,64);
		else
			glColor4ub(0,196,255,64);
		
		for (int j = 0; j < pd->ntris; ++j)
		{
			const unsigned char* t = &header->dtris[(pd->tbase+j)*4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->nv)
					glVertex3fv(&header->verts[p->v[t[k]]*3]);
				else
					glVertex3fv(&header->dverts[(pd->vbase+t[k]-p->nv)*3]);
			}
		}
	}
	glEnd();
	
	// Draw inter poly boundaries
	glColor4ub(0,48,64,32);
	glLineWidth(1.5f);
	
	drawPolyBoundaries(header, true);
	
	// Draw outer poly boundaries
	glLineWidth(2.5f);
	glColor4ub(0,48,64,220);
	
	drawPolyBoundaries(header, false);
	
	glLineWidth(1.0f);
	
	glPointSize(3.0f);
	glColor4ub(0,0,0,196);
	glBegin(GL_POINTS);
	for (int i = 0; i < header->nverts; ++i)
	{
		const float* v = &header->verts[i*3];
		glVertex3f(v[0], v[1], v[2]);
	}
	glEnd();
	glPointSize(1.0f);	
	
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

void dtDebugDrawNavMesh(const dtNavMesh* mesh, bool drawClosedList)
{
	if (!mesh) return;
	
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile->header) continue;
		drawMeshTile(mesh, tile, drawClosedList);
	}
}


static void drawMeshTileBVTree(const dtNavMesh* mesh, const dtMeshTile* tile)
{
	const dtMeshHeader* header = tile->header;
	
	// Draw BV nodes.
	const float col[] = { 1,1,1,0.5f };
	const float cs = 1.0f / header->bvquant;
	glBegin(GL_LINES);
	for (int i = 0; i < header->nbvtree; ++i)
	{
		const dtBVNode* n = &header->bvtree[i];
		if (n->i < 0) // Leaf indices are positive.
			continue;
		drawBoxWire(header->bmin[0] + n->bmin[0]*cs,
		header->bmin[1] + n->bmin[1]*cs,
		header->bmin[2] + n->bmin[2]*cs,
		header->bmin[0] + n->bmax[0]*cs,
		header->bmin[1] + n->bmax[1]*cs,
		header->bmin[2] + n->bmax[2]*cs, col);
	}
	glEnd();
	
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

void dtDebugDrawNavMeshBVTree(const dtNavMesh* mesh)
{
	if (!mesh) return;
	
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile->header) continue;
		drawMeshTileBVTree(mesh, tile);
	}
}

void dtDebugDrawNavMeshPoly(const dtNavMesh* mesh, dtPolyRef ref, const float* col)
{
	int ip = 0;
	const dtMeshTile* tile = mesh->getTileByRef(ref, &ip);
	if (!tile)
		return;
	const dtMeshHeader* header = tile->header;
	const dtPoly* p = &header->polys[ip];
	const dtPolyDetail* pd = &header->dmeshes[ip];
	
	glColor4f(col[0],col[1],col[2],0.25f);
	
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < pd->ntris; ++i)
	{
		const unsigned char* t = &header->dtris[(pd->tbase+i)*4];
		for (int j = 0; j < 3; ++j)
		{
			if (t[j] < p->nv)
				glVertex3fv(&header->verts[p->v[t[j]]*3]);
			else
				glVertex3fv(&header->dverts[(pd->vbase+t[j]-p->nv)*3]);
		}
	}
	glEnd();
}

