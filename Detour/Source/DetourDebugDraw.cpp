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
#include "DetourStatNavMesh.h"
#include "SDL.h"
#include "SDL_Opengl.h"

void dtDebugDrawStatNavMeshPoly(const dtStatNavMesh* mesh, dtPolyRef ref, const float* col)
{
	const dtPoly* p = mesh->getPolyByRef(ref);
	if (!p)
		return;
	glColor4f(col[0],col[1],col[2],0.25f);
	glBegin(GL_TRIANGLES);
	unsigned short vi[3];
	for (int j = 2; j < (int)p->nv; ++j)
	{
		vi[0] = p->v[0];
		vi[1] = p->v[j-1];
		vi[2] = p->v[j];
		for (int k = 0; k < 3; ++k)
		{
			const float* v = mesh->getVertex(vi[k]);
			glVertex3f(v[0], v[1]+0.2f, v[2]);
		}
	}
	glEnd();
}

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

void dtDebugDrawStatNavMeshBVTree(const dtStatNavMesh* mesh)
{
	const float col[] = { 1,1,1,0.5f };
	const dtStatNavMeshHeader* hdr = mesh->getHeader();
	
	const dtBVNode* nodes = mesh->getBvTreeNodes();
	int nnodes = mesh->getBvTreeNodeCount();
	
	glBegin(GL_LINES);

	for (int i = 0; i < nnodes; ++i)
	{
		const dtBVNode* n = &nodes[i];
		if (n->i < 0) // Leaf indices are positive.
			continue;
		drawBoxWire(hdr->bmin[0] + n->bmin[0]*hdr->cs,
					hdr->bmin[1] + n->bmin[1]*hdr->cs,
					hdr->bmin[2] + n->bmin[2]*hdr->cs,
					hdr->bmin[0] + n->bmax[0]*hdr->cs,
					hdr->bmin[1] + n->bmax[1]*hdr->cs,
					hdr->bmin[2] + n->bmax[2]*hdr->cs, col);
	}
	glEnd();
}

void dtDebugDrawStatNavMesh(const dtStatNavMesh* mesh)
{
	glColor4ub(0,196,255,64);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < mesh->getPolyCount(); ++i)
	{
		const dtPoly* p = mesh->getPoly(i);
		unsigned short vi[3];
		for (int j = 2; j < (int)p->nv; ++j)
		{
			vi[0] = p->v[0];
			vi[1] = p->v[j-1];
			vi[2] = p->v[j];
			for (int k = 0; k < 3; ++k)
			{
				const float* v = mesh->getVertex(vi[k]);
				glVertex3f(v[0], v[1]+0.2f, v[2]);
			}
		}
	}
	glEnd();
	
	// Draw tri boundaries
	glColor4ub(0,0,0,64);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (int i = 0; i < mesh->getPolyCount(); ++i)
	{
		const dtPoly* p = mesh->getPoly(i);
		for (int j = 0, nj = (int)p->nv; j < nj; ++j)
		{
			if (p->n[j] == 0) continue;
			int vi[2];
			vi[0] = p->v[j];
			vi[1] = p->v[(j+1) % nj];
			for (int k = 0; k < 2; ++k)
			{
				const float* v = mesh->getVertex(vi[k]);
				glVertex3f(v[0], v[1]+0.21f, v[2]);
			}
		}
	}
	glEnd();
	
	// Draw boundaries
	glLineWidth(3.0f);
	glColor4ub(0,0,0,128);
	glBegin(GL_LINES);
	for (int i = 0; i < mesh->getPolyCount(); ++i)
	{
		const dtPoly* p = mesh->getPoly(i);
		for (int j = 0, nj = (int)p->nv; j < nj; ++j)
		{
			if (p->n[j] != 0) continue;
			int vi[2];
			vi[0] = p->v[j];
			vi[1] = p->v[(j+1) % nj];
			for (int k = 0; k < 2; ++k)
			{
				const float* v = mesh->getVertex(vi[k]);
				glVertex3f(v[0], v[1]+0.21f, v[2]);
			}
		}
	}
	glEnd();
	glLineWidth(1.0f);
	
	glPointSize(4.0f);
	glColor4ub(0,0,0,128);
	glBegin(GL_POINTS);
	for (int i = 0; i < mesh->getVertexCount(); ++i)
	{
		const float* v = mesh->getVertex(i);
		glVertex3f(v[0], v[1]+0.21f, v[2]);
	}
	glEnd();
	glPointSize(1.0f);	
}
