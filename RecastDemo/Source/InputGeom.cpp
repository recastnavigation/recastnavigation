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

#define _USE_MATH_DEFINES
#include <math.h>
#include "Recast.h"
#include "RecastLog.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"


static bool intersectSegmentTriangle(const float* sp, const float* sq,
									 const float* a, const float* b, const float* c,
									 float &t)
{
	float v, w;
	float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	vsub(ab, b, a);
	vsub(ac, c, a);
	vsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	vcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = vdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	vsub(ap, sp, a);
	t = vdot(ap, norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	vcross(e, qp, ap);
	v = vdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -vdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}



InputGeom::InputGeom() :
	m_chunkyMesh(0),
	m_mesh(0),
	m_nlinks(0)
{
}

InputGeom::~InputGeom()
{
	delete m_chunkyMesh;
	delete m_mesh;
}
		
bool InputGeom::loadMesh(const char* filepath)
{
	if (m_mesh)
	{
		delete m_chunkyMesh;
		m_chunkyMesh = 0;
		delete m_mesh;
		m_mesh = 0;
		m_nlinks = 0;
	}
	
	m_mesh = new rcMeshLoaderObj;
	if (!m_mesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "loadMesh: Out of memory 'm_mesh'.");
		return false;
	}
	if (!m_mesh->load(filepath))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
		return false;
	}

	rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), m_meshBMin, m_meshBMax);

	m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, m_chunkyMesh))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
		return false;
	}		

	return true;
}
		
bool InputGeom::raycastMesh(float* src, float* dst, float& tmin)
{
	float dir[3];
	vsub(dir, dst, src);
	
	int nt = m_mesh->getTriCount();
	const float* verts = m_mesh->getVerts();
	const float* normals = m_mesh->getNormals();
	const int* tris = m_mesh->getTris();
	tmin = 1.0f;
	bool hit = false;
	
	for (int i = 0; i < nt*3; i += 3)
	{
		const float* n = &normals[i];
		if (vdot(dir, n) > 0)
			continue;
		
		float t = 1;
		if (intersectSegmentTriangle(src, dst,
									 &verts[tris[i]*3],
									 &verts[tris[i+1]*3],
									 &verts[tris[i+2]*3], t))
		{
			if (t < tmin)
				tmin = t;
			hit = true;
		}
	}
	
	return hit;
}

void InputGeom::addOffMeshLink(const float* spos, const float* epos)
{
	if (m_nlinks >= MAX_LINKS) return;
	float* v = &m_linkVerts[m_nlinks*3*2];
	m_nlinks++;
	vcopy(&v[0], spos);
	vcopy(&v[3], epos);
}

void InputGeom::deleteOffMeshLink(int i)
{
	m_nlinks--;
	float* src = &m_linkVerts[(m_nlinks-1)*3*2];
	float* dst = &m_linkVerts[i*3*2];
	vcopy(&dst[0], &src[0]);
	vcopy(&dst[3], &src[3]);
}

void InputGeom::drawLinks(duDebugDraw* dd, const float s)
{
	for (int i = 0; i < m_nlinks; ++i)
	{
		float* v = &m_linkVerts[i*3*2];
		duDebugDrawArc(dd, v[0],v[1],v[2], v[3],v[4],v[5], 0.25f, duRGBA(255,255,255,192), 1.0f);
		duDebugDrawCross(dd, v[0],v[1]+0.1f,v[2], s, duRGBA(0,0,0,255), 2.0f);
		duDebugDrawCross(dd, v[3],v[4]+0.1f,v[5], s, duRGBA(0,0,0,255), 2.0f);
	}	
}
