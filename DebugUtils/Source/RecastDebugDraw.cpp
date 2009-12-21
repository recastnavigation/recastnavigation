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
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "Recast.h"

inline unsigned int dark(unsigned int col)
{
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000);
}

void duDebugDrawTriMesh(duDebugDraw* dd, const float* verts, int nverts,
						const int* tris, const float* normals, int ntris,
						const unsigned char* flags)
{
	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		unsigned int color;
		unsigned char a = (unsigned char)(150*(2+normals[i+0]+normals[i+1])/4);
		if (flags && !flags[i/3])
			color = duRGBA(a,a/4,a/16,255);
		else
			color = duRGBA(a,a,a,255);
			
		dd->vertex(&verts[tris[i+0]*3], color);
		dd->vertex(&verts[tris[i+1]*3], color);
		dd->vertex(&verts[tris[i+2]*3], color);
	}
	dd->end();
}

void duDebugDrawTriMeshSlope(duDebugDraw* dd, const float* verts, int nverts,
							 const int* tris, const float* normals, int ntris,
							 const float walkableSlopeAngle)
{
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*(float)M_PI);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		const float* norm = &normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(255*(2+normals[i+0]+normals[i+1])/4);
		if (norm[1] < walkableThr)
			color = duRGBA(a,a/4,a/16,255);
		else
			color = duRGBA(a,a,a,255);
		
		dd->vertex(&verts[tris[i+0]*3], color);
		dd->vertex(&verts[tris[i+1]*3], color);
		dd->vertex(&verts[tris[i+2]*3], color);
	}
	dd->end();
}

static void drawBoxWire(duDebugDraw* dd,
						float minx, float miny, float minz,
						float maxx, float maxy, float maxz,
						const float* col)
{
	// Submits 24 vertices.

	unsigned int color = duRGBAf(col[0],col[1],col[2],col[3]);
	
	// Top
	dd->vertex(minx, miny, minz, color);
	dd->vertex(maxx, miny, minz, color);
	dd->vertex(maxx, miny, minz, color);
	dd->vertex(maxx, miny, maxz, color);
	dd->vertex(maxx, miny, maxz, color);
	dd->vertex(minx, miny, maxz, color);
	dd->vertex(minx, miny, maxz, color);
	dd->vertex(minx, miny, minz, color);
	
	// bottom
	dd->vertex(minx, maxy, minz, color);
	dd->vertex(maxx, maxy, minz, color);
	dd->vertex(maxx, maxy, minz, color);
	dd->vertex(maxx, maxy, maxz, color);
	dd->vertex(maxx, maxy, maxz, color);
	dd->vertex(minx, maxy, maxz, color);
	dd->vertex(minx, maxy, maxz, color);
	dd->vertex(minx, maxy, minz, color);
	
	// Sides
	dd->vertex(minx, miny, minz, color);
	dd->vertex(minx, maxy, minz, color);
	dd->vertex(maxx, miny, minz, color);
	dd->vertex(maxx, maxy, minz, color);
	dd->vertex(maxx, miny, maxz, color);
	dd->vertex(maxx, maxy, maxz, color);
	dd->vertex(minx, miny, maxz, color);
	dd->vertex(minx, maxy, maxz, color);
}

static void drawBox(duDebugDraw* dd,
					float minx, float miny, float minz,
					float maxx, float maxy, float maxz,
					const float* col1, const float* col2)
{
	// Submits 24 vertices.
	
	const float verts[8*3] =
	{
		minx, miny, minz,
		maxx, miny, minz,
		maxx, miny, maxz,
		minx, miny, maxz,
		minx, maxy, minz,
		maxx, maxy, minz,
		maxx, maxy, maxz,
		minx, maxy, maxz,
	};
	static const float dim[6] =
	{
		0.95f, 0.55f, 0.65f, 0.85f, 0.65f, 0.85f, 
	};
	static const unsigned char inds[6*5] =
	{
		0,  7, 6, 5, 4,
		1,  0, 1, 2, 3,
		2,  1, 5, 6, 2,
		3,  3, 7, 4, 0,
		4,  2, 6, 7, 3,
		5,  0, 4, 5, 1,
	};
	
	const unsigned char* in = inds;
	for (int i = 0; i < 6; ++i)
	{
		float d = dim[*in]; in++;
		unsigned int color;
		if (i == 0)
			color = duRGBAf(d*col2[0],d*col2[1],d*col2[2], col2[3]);
		else
			color = duRGBAf(d*col1[0],d*col1[1],d*col1[2], col1[3]);
		dd->vertex(&verts[*in*3], color); in++;
		dd->vertex(&verts[*in*3], color); in++;
		dd->vertex(&verts[*in*3], color); in++;
		dd->vertex(&verts[*in*3], color); in++;
	}
}

void duDebugDrawCylinderWire(duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz,
							 const float* col)
{
	static const int NUM_SEG = 16;
	float dir[NUM_SEG*2];
	for (int i = 0; i < NUM_SEG; ++i)
	{
		const float a = (float)i/(float)NUM_SEG*(float)M_PI*2;
		dir[i*2] = cosf(a);
		dir[i*2+1] = sinf(a);
	}

	const float cx = (maxx + minx)/2;
	const float cz = (maxz + minz)/2;
	const float rx = (maxx - minx)/2;
	const float rz = (maxz - minz)/2;
	
	unsigned int color = duRGBAf(col[0],col[1],col[2],col[3]);
	
	dd->begin(DU_DRAW_LINES);
	
	for (int i = 0, j=NUM_SEG-1; i < NUM_SEG; j=i++)
	{
		dd->vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, color);
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, color);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, color);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, color);
	}
	for (int i = 0; i < NUM_SEG; i += NUM_SEG/4)
	{
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, color);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, color);
	}
	
	dd->end();
}

void duDebugDrawBoxWire(duDebugDraw* dd, float minx, float miny, float minz, float maxx, float maxy, float maxz, const float* col)
{
	dd->begin(DU_DRAW_LINES, 1.0f);
	drawBoxWire(dd, minx, miny, minz, maxx, maxy, maxz, col);
	dd->end();
}

void duDebugDrawBox(duDebugDraw* dd, float minx, float miny, float minz, float maxx, float maxy, float maxz,
					const float* col1, const float* col2)
{
	dd->begin(DU_DRAW_QUADS,24);
	drawBox(dd, minx, miny, minz, maxx, maxy, maxz, col1, col2);
	dd->end();
}

static int getSpanCount(const rcHeightfield& hf)
{
	const int w = hf.width;
	const int h = hf.height;
	int spanCount = 0;
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
			for (rcSpan* s = hf.spans[x + y*w]; s; s = s->next)
					spanCount++;
	return spanCount;
}

void duDebugDrawHeightfieldSolid(duDebugDraw* dd, const rcHeightfield& hf)
{
	static const float col0[4] = { 1,1,1,1 };
	
	const float* orig = hf.bmin;
	const float cs = hf.cs;
	const float ch = hf.ch;
	
	const int w = hf.width;
	const int h = hf.height;
		
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig[0] + x*cs;
			float fz = orig[2] + y*cs;
			const rcSpan* s = hf.spans[x + y*w];
			while (s)
			{
				drawBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, col0, col0);
				s = s->next;
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldWalkable(duDebugDraw* dd, const rcHeightfield& hf)
{
	static const float colb[4] = {0.85f,0.85f,0.85f,1 }; // Base
	static const float col0[4] = {0.5f, 0.75f, 0.85f,1}; // Culled
	static const float col1[4] = {0.3f, 0.55f, 0.65f, 1}; // Walkable
	static const float col2[4] = {0.15f, 0.4f, 0.5f,1}; // Ledge
	
	const float* orig = hf.bmin;
	const float cs = hf.cs;
	const float ch = hf.ch;
	
	const int w = hf.width;
	const int h = hf.height;
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig[0] + x*cs;
			float fz = orig[2] + y*cs;
			const rcSpan* s = hf.spans[x + y*w];
			while (s)
			{
				const float* c = col0;
				if (s->flags & RC_LEDGE)
					c = col2;
				else if (s->flags & RC_WALKABLE)
					c = col1;
				drawBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, colb, c);
				s = s->next;
			}
		}
	}
	
	dd->end();
}

void duDebugDrawCompactHeightfieldSolid(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	const float cs = chf.cs;
	const float ch = chf.ch;

	unsigned int color = duRGBA(0,192,255,64);
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];

			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

void duDebugDrawCompactHeightfieldRegions(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	const float cs = chf.cs;
	const float ch = chf.ch;

	dd->begin(DU_DRAW_QUADS);

	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y)*ch;
				unsigned int color;
				if (chf.reg[i])
					color = duIntToCol(chf.reg[i], 192);
				else
					color = duRGBA(0,0,0,64);

				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	
	dd->end();
}


void duDebugDrawCompactHeightfieldDistance(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	const float cs = chf.cs;
	const float ch = chf.ch;
			
	float maxd = chf.maxDistance;
	if (maxd < 1.0f) maxd = 1;
	const float dscale = 255.0f / maxd;
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				const unsigned char cd = (unsigned char)(chf.dist[i] * dscale);
				const unsigned int color = duRGBA(cd,cd,cd,255);
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

static void getContourCenter(const rcContour* cont, const float* orig, float cs, float ch, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	if (!cont->nverts)
		return;
	for (int i = 0; i < cont->nverts; ++i)
	{
		const int* v = &cont->verts[i*4];
		center[0] += (float)v[0];
		center[1] += (float)v[1];
		center[2] += (float)v[2];
	}
	const float s = 1.0f / cont->nverts;
	center[0] *= s * cs;
	center[1] *= s * ch;
	center[2] *= s * cs;
	center[0] += orig[0];
	center[1] += orig[1] + 4*ch;
	center[2] += orig[2];
}

static const rcContour* findContourFromSet(const rcContourSet& cset, unsigned short reg)
{
	for (int i = 0; i < cset.nconts; ++i)
	{
		if (cset.conts[i].reg == reg)
			return &cset.conts[i];
	}
	return 0;
}

static const int NUM_ADU_PTS = 8;

static void drawArc(duDebugDraw* dd, const float* p0, const float* p1, unsigned int color)
{
	// Submits NPTS*2 vertices.
	float pts[NUM_ADU_PTS*3];
	float dir[3];
	vsub(dir, p1, p0);
	const float len = sqrtf(vdistSqr(p0, p1));
	for (int i = 0; i < NUM_ADU_PTS; ++i)
	{
		float u = (float)i / (float)(NUM_ADU_PTS-1);
		float* p = &pts[i*3];
		p[0] = p0[0] + dir[0] * u;
		p[1] = p0[1] + dir[1] * u + (len/4) * (1-rcSqr(u*2-1));
		p[2] = p0[2] + dir[2] * u;
	}
	for (int i = 0; i < NUM_ADU_PTS-1; ++i)
	{
		dd->vertex(&pts[i*3], color);
		dd->vertex(&pts[(i+1)*3], color);
	}
}

void duDebugDrawArc(duDebugDraw* dd, const float* p0, const float* p1, const float* col, float lineWidth)
{
	const unsigned int color = duRGBAf(col[0],col[1],col[2],col[3]);
	dd->begin(DU_DRAW_LINES, lineWidth);
	drawArc(dd, p0, p1, color);
	dd->end();
}

void duDebugDrawRegionConnections(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	// Draw centers
	float pos[3], pos2[3];

	unsigned int color = duRGBA(0,0,0,196);

	dd->begin(DU_DRAW_LINES, 2.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour* cont = &cset.conts[i];
		getContourCenter(cont, orig, cs, ch, pos);
		for (int j = 0; j < cont->nverts; ++j)
		{
			const int* v = &cont->verts[j*4];
			if (v[3] == 0 || (unsigned short)v[3] < cont->reg) continue;
			const rcContour* cont2 = findContourFromSet(cset, (unsigned short)v[3]);
			if (cont2)
			{
				getContourCenter(cont2, orig, cs, ch, pos2);
				drawArc(dd, pos, pos2, color);
			}
		}
	}
	
	dd->end();

	unsigned char a = (unsigned char)(alpha * 255.0f);

	dd->begin(DU_DRAW_POINTS, 7.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour* cont = &cset.conts[i];
		unsigned int color = dark(duIntToCol(cont->reg,a));
		getContourCenter(cont, orig, cs, ch, pos);
		dd->vertex(pos, color);
	}
	dd->end();
}

void duDebugDrawRawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.0f);
			
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duIntToCol(c.reg, a);

		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz,color);
			if (j > 0)
				dd->vertex(fx,fy,fz,color);
		}
		// Loop last segment.
		const int* v = &c.rverts[0];
		float fx = orig[0] + v[0]*cs;
		float fy = orig[1] + (v[1]+1+(i&1))*ch;
		float fz = orig[2] + v[2]*cs;
		dd->vertex(fx,fy,fz,color);
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 2.0f);	

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = dark(duIntToCol(c.reg, a));
		
		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}
			
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.5f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		if (!c.nverts)
			continue;
		unsigned int color = duIntToCol(c.reg, a);

		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, color);
			if (j > 0)
				dd->vertex(fx,fy,fz, color);
		}
		// Loop last segment
		const int* v = &c.verts[0];
		float fx = orig[0] + v[0]*cs;
		float fy = orig[1] + (v[1]+1+(i&1))*ch;
		float fz = orig[2] + v[2]*cs;
		dd->vertex(fx,fy,fz, color);
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = dark(duIntToCol(c.reg, a));
		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}

			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawPolyMesh(duDebugDraw* dd, const struct rcPolyMesh& mesh)
{
	const int nvp = mesh.nvp;
	const float cs = mesh.cs;
	const float ch = mesh.ch;
	const float* orig = mesh.bmin;
	
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		unsigned int color = duIntToCol(i, 192);
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();

	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			if (p[nvp+j] == 0xffff) continue;
			int vi[2];
			vi[0] = p[j];
			if (j+1 >= nvp || p[j+1] == 0xffff)
				vi[1] = p[0];
			else
				vi[1] = p[j+1];
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == 0xffff) break;
			if (p[nvp+j] != 0xffff) continue;
			int vi[2];
			vi[0] = p[j];
			if (j+1 >= nvp || p[j+1] == 0xffff)
				vi[1] = p[0];
			else
				vi[1] = p[j+1];
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, colb);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < mesh.nverts; ++i)
	{
		const unsigned short* v = &mesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}

void duDebugDrawPolyMeshDetail(duDebugDraw* dd, const struct rcPolyMeshDetail& dmesh)
{
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short btris = m[2];
		const unsigned short ntris = m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];

		unsigned int color = duIntToCol(i, 192);

		for (int j = 0; j < ntris; ++j)
		{
			dd->vertex(&verts[tris[j*4+0]*3], color);
			dd->vertex(&verts[tris[j*4+1]*3], color);
			dd->vertex(&verts[tris[j*4+2]*3], color);
		}
	}
	dd->end();

	// Internal edges.
	dd->begin(DU_DRAW_LINES, 1.0f);
	const unsigned int coli = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short btris = m[2];
		const unsigned short ntris = m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef == 0)
				{
					// Internal edge
					if (t[kp] < t[k])
					{
						dd->vertex(&verts[t[kp]*3], coli);
						dd->vertex(&verts[t[k]*3], coli);
					}
				}
			}
		}
	}
	dd->end();
	
	// External edges.
	dd->begin(DU_DRAW_LINES, 2.0f);
	const unsigned int cole = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short btris = m[2];
		const unsigned short ntris = m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef != 0)
				{
					// Ext edge
					dd->vertex(&verts[t[kp]*3], cole);
					dd->vertex(&verts[t[k]*3], cole);
				}
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short nverts = m[1];
		const float* verts = &dmesh.verts[bverts*3];
		for (int j = 0; j < nverts; ++j)
			dd->vertex(&verts[j*3], colv);
	}
	dd->end();
}
