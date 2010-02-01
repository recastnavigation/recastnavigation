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
#include <stdio.h>
#include <ctype.h>
#include "Recast.h"
#include "RecastLog.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"

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

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}



InputGeom::InputGeom() :
	m_chunkyMesh(0),
	m_mesh(0),
	m_offMeshConCount(0),
	m_boxVolCount(0)
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
	}
	m_offMeshConCount = 0;
	m_boxVolCount = 0;
	
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

bool InputGeom::load(const char* filePath)
{
	char* buf = 0;
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
		return false;
	fseek(fp, 0, SEEK_END);
	int bufSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}
	fread(buf, bufSize, 1, fp);
	fclose(fp);
	
	m_offMeshConCount = 0;
	m_boxVolCount = 0;
	delete m_mesh;
	m_mesh = 0;

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		if (row[0] == 'f')
		{
			// File name.
			const char* name = row+1;
			// Skip white spaces
			while (*name && isspace(*name))
				name++;
			if (*name)
			{
				if (!loadMesh(name))
				{
					delete [] buf;
					return false;
				}
			}
		}
		else if (row[0] == 'c')
		{
			// Off-mesh connection
			if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
			{
				float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
				int bidir;
				float rad;
				sscanf(row+1, "%f %f %f  %f %f %f %f %d",
					   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rad, &bidir);
				m_offMeshConRads[m_offMeshConCount] = rad;
				m_offMeshConDirs[m_offMeshConCount] = bidir;
				m_offMeshConCount++;
			}
		}
		else if (row[0] == 'b')
		{
			// Box volumes
			if (m_boxVolCount < MAX_BOX_VOLUMES)
			{
				float* v = &m_boxVolVerts[m_boxVolCount*3*2];
				int type;
				sscanf(row+1, "%f %f %f  %f %f %f %d",
					   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &type);
				m_boxVolTypes[m_boxVolCount] = (unsigned char)type;
				m_boxVolCount++;
			}
		}
	}
	
	delete [] buf;
	
	return true;
}

bool InputGeom::save(const char* filepath)
{
	if (!m_mesh) return false;
	
	FILE* fp = fopen(filepath, "w");
	if (!fp) return false;
	
	// Store mesh filename.
	fprintf(fp, "f %s\n", m_mesh->getFileName());
	
	// Store off-mesh links.
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		const float* v = &m_offMeshConVerts[i*3*2];
		const float rad = m_offMeshConRads[i];
		const int bidir = m_offMeshConDirs[i];
		fprintf(fp, "c %f %f %f  %f %f %f  %f %d\n",
				v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir);
	}

	// Box volumes
	for (int i = 0; i < m_boxVolCount; ++i)
	{
		const float* v = &m_boxVolVerts[i*3*2];
		const int bidir = m_boxVolTypes[i];
		fprintf(fp, "b %f %f %f  %f %f %f  %d\n",
				v[0], v[1], v[2], v[3], v[4], v[5], bidir);
	}
	
	fclose(fp);
	
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

void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad, unsigned char bidir)
{
	if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
	float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
	m_offMeshConRads[m_offMeshConCount] = rad;
	m_offMeshConDirs[m_offMeshConCount] = bidir;
	vcopy(&v[0], spos);
	vcopy(&v[3], epos);
	m_offMeshConCount++;
}

void InputGeom::deleteOffMeshConnection(int i)
{
	m_offMeshConCount--;
	float* src = &m_offMeshConVerts[m_offMeshConCount*3*2];
	float* dst = &m_offMeshConVerts[i*3*2];
	vcopy(&dst[0], &src[0]);
	vcopy(&dst[3], &src[3]);
	m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
	m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, bool hilight)
{
	unsigned int conColor = duRGBA(192,0,128,192);
	unsigned int baseColor = duRGBA(0,0,0,64);
	dd->depthMask(false);

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		float* v = &m_offMeshConVerts[i*3*2];

		dd->vertex(v[0],v[1],v[2], baseColor);
		dd->vertex(v[0],v[1]+0.2f,v[2], baseColor);
		
		dd->vertex(v[3],v[4],v[5], baseColor);
		dd->vertex(v[3],v[4]+0.2f,v[5], baseColor);
		
		duAppendCircle(dd, v[0],v[1]+0.1f,v[2], m_offMeshConRads[i], baseColor);
		duAppendCircle(dd, v[3],v[4]+0.1f,v[5], m_offMeshConRads[i], baseColor);

		if (hilight)
		{
			duAppendArc(dd, v[0],v[1],v[2], v[3],v[4],v[5], 0.25f,
						(m_offMeshConDirs[i]&1) ? 0.6f : 0.0f, 0.6f, conColor);
		}
	}	
	dd->end();

	dd->depthMask(true);
}

void InputGeom::addBoxVolume(const float* bmin, const float* bmax, unsigned char type)
{
	if (m_boxVolCount >= MAX_OFFMESH_CONNECTIONS) return;
	float* v = &m_boxVolVerts[m_boxVolCount*3*2];
	m_boxVolTypes[m_boxVolCount] = type;
	vcopy(&v[0], bmin);
	vcopy(&v[3], bmax);
	m_boxVolCount++;
}

void InputGeom::deleteBoxVolume(int i)
{
	m_boxVolCount--;
	float* src = &m_boxVolVerts[m_boxVolCount*3*2];
	float* dst = &m_boxVolVerts[i*3*2];
	vcopy(&dst[0], &src[0]);
	vcopy(&dst[3], &src[3]);
	m_boxVolTypes[i] = m_boxVolTypes[m_boxVolCount];
}

void InputGeom::drawBoxVolumes(struct duDebugDraw* dd, bool hilight)
{
	dd->depthMask(false);
	
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_boxVolCount; ++i)
	{
		unsigned int col = duIntToCol(m_boxVolTypes[i]+1, 220);
		const float* bounds = &m_boxVolVerts[i*3*2];
		duAppendBoxWire(dd, bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5], col); 
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 4.0f);
	for (int i = 0; i < m_boxVolCount; ++i)
	{
		unsigned int col = duDarkenColor(duIntToCol(m_boxVolTypes[i]+1, 255));
		const float* bounds = &m_boxVolVerts[i*3*2];
		duAppendBoxPoints(dd, bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5], col); 
	}
	dd->end();
	
	dd->depthMask(true);
}
