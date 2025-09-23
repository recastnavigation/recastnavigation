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
#include "RecastDebugDraw.h"
#include "Recast.h"

static float duGetXYZByIndex(const Vector3& v, int idx)
{
	switch (idx)
	{
	case 1:
		return v.y;
	case 2:
		return v.z;
	}

	return v.x;
}

void duDebugDrawTriMesh(struct duDebugDraw* dd, const std::vector<Vector3>& verts, const std::vector<Triangle>& tris, const std::vector<Vector3>& normals, const unsigned char* flags, const float texScale)
//void duDebugDrawTriMesh(duDebugDraw* dd, const float* verts, int /*nverts*/,
//						const int* tris, const float* normals, int ntris,
//						const unsigned char* flags, const float texScale)
{
	if (!dd) return;
	//if (!verts) return;
	//if (!tris) return;
	//if (!normals) return;

	Vector2 uva, uvb, uvc;

	const unsigned int unwalkable = duRGBA(192,128,0,255);

	dd->texture(true);

	dd->begin(DU_DRAW_TRIS);

	for (size_t i = 0; i < tris.size(); ++i)
	{
		const auto& tri = tris[i];
		const auto& norm = normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220 * (2 + norm.x + norm.y) / 4);
		if (flags && !flags[i / 3])
			color = duLerpCol(duRGBA(a, a, a, 255), unwalkable, 64);
		else
			color = duRGBA(a, a, a, 255);


		const auto& va = verts[tri.v0];
		const auto& vb = verts[tri.v1];
		const auto& vc = verts[tri.v2];
		//const float* va = &verts[tris[i + 0] * 3];
		//const float* vb = &verts[tris[i + 1] * 3];
		//const float* vc = &verts[tris[i + 2] * 3];

		int ax = 0, ay = 0;
		if (rcAbs(norm.y) > rcAbs(norm.x))
		{
			ax = 1;
			if (rcAbs(norm.z) > rcAbs(norm.y))
				ax = 2;
		}
		else
		{
			if (rcAbs(norm.z) > rcAbs(norm.x))
				ax = 2;
		}
		
		ax = (1 << ax) & 3; // +1 mod 3
		ay = (1 << ax) & 3; // +1 mod 3

		uva.x = duGetXYZByIndex(va, ax) * texScale;
		uva.y = duGetXYZByIndex(va, ay) * texScale;
		uvb.x = duGetXYZByIndex(vb, ax) * texScale;
		uvb.y = duGetXYZByIndex(vb, ay) * texScale;
		uvc.x = duGetXYZByIndex(vc, ax) * texScale;
		uvc.y = duGetXYZByIndex(vc, ay) * texScale;

		dd->vertex(va.x, va.y, va.z, color, uva.x, uva.y);
		dd->vertex(va.x, va.y, va.z, color, uvb.x, uvb.y);
		dd->vertex(va.x, va.y, va.z, color, uvc.x, uvc.y);
	}

	//for (int i = 0; i < ntris*3; i += 3)
	//{
	//	const float* norm = &normals[i];
	//	unsigned int color;
	//	unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
	//	if (flags && !flags[i/3])
	//		color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
	//	else
	//		color = duRGBA(a,a,a,255);

	//	const float* va = &verts[tris[i+0]*3];
	//	const float* vb = &verts[tris[i+1]*3];
	//	const float* vc = &verts[tris[i+2]*3];
	//	
	//	int ax = 0, ay = 0;
	//	if (rcAbs(norm[1]) > rcAbs(norm[ax]))
	//		ax = 1;
	//	if (rcAbs(norm[2]) > rcAbs(norm[ax]))
	//		ax = 2;
	//	ax = (1<<ax)&3; // +1 mod 3
	//	ay = (1<<ax)&3; // +1 mod 3

	//	uva[0] = va[ax]*texScale;
	//	uva[1] = va[ay]*texScale;
	//	uvb[0] = vb[ax]*texScale;
	//	uvb[1] = vb[ay]*texScale;
	//	uvc[0] = vc[ax]*texScale;
	//	uvc[1] = vc[ay]*texScale;
	//	
	//	dd->vertex(va, color, uva);
	//	dd->vertex(vb, color, uvb);
	//	dd->vertex(vc, color, uvc);
	//}
	dd->end();
	dd->texture(false);
}

void duDebugDrawTriMeshSlope(struct duDebugDraw* dd, const std::vector<Vector3>& verts, const std::vector<Triangle>& tris, const std::vector<Vector3>& normals, const float walkableSlopeAngle, const float texScale)
//void duDebugDrawTriMeshSlope(duDebugDraw* dd, const float* verts, int /*nverts*/,
//							 const int* tris, const float* normals, int ntris,
//							 const float walkableSlopeAngle, const float texScale)
{
	if (!dd) return;
	//if (!verts) return;
	//if (!tris) return;
	//if (!normals) return;
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*DU_PI);
	
	//float uva[2];
	//float uvb[2];
	//float uvc[2];
	Vector2 uva, uvb, uvc;

	dd->texture(true);

	const unsigned int unwalkable = duRGBA(192,128,0,255);
	
	dd->begin(DU_DRAW_TRIS);

	for (size_t i = 0; i < tris.size(); ++i)
	{
		const auto& tri = tris[i];
		const auto& norm = normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220 * (2 + norm.x + norm.y) / 4);
		if (norm.y < walkableThr)
			color = duLerpCol(duRGBA(a, a, a, 255), unwalkable, 64);
		else
			color = duRGBA(a, a, a, 255);


		const auto& va = verts[tri.v0];
		const auto& vb = verts[tri.v1];
		const auto& vc = verts[tri.v2];
		//const float* va = &verts[tris[i + 0] * 3];
		//const float* vb = &verts[tris[i + 1] * 3];
		//const float* vc = &verts[tris[i + 2] * 3];

		int ax = 0, ay = 0;
		if (rcAbs(norm.y) > rcAbs(norm.x))
		{
			ax = 1;
			if (rcAbs(norm.z) > rcAbs(norm.y))
				ax = 2;
		}
		else
		{
			if (rcAbs(norm.z) > rcAbs(norm.x))
				ax = 2;
		}

		ax = (1 << ax) & 3; // +1 mod 3
		ay = (1 << ax) & 3; // +1 mod 3

		uva.x = duGetXYZByIndex(va, ax) * texScale;
		uva.y = duGetXYZByIndex(va, ay) * texScale;
		uvb.x = duGetXYZByIndex(vb, ax) * texScale;
		uvb.y = duGetXYZByIndex(vb, ay) * texScale;
		uvc.x = duGetXYZByIndex(vc, ax) * texScale;
		uvc.y = duGetXYZByIndex(vc, ay) * texScale;

		dd->vertex(va.x, va.y, va.z, color, uva.x, uva.y);
		dd->vertex(vb.x, vb.y, vb.z, color, uvb.x, uvb.y);
		dd->vertex(vc.x, vc.y, vc.z, color, uvc.x, uvc.y);
	}


	//for (int i = 0; i < ntris*3; i += 3)
	//{
	//	const float* norm = &normals[i];
	//	unsigned int color;
	//	unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
	//	if (norm[1] < walkableThr)
	//		color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
	//	else
	//		color = duRGBA(a,a,a,255);
	//	
	//	const float* va = &verts[tris[i+0]*3];
	//	const float* vb = &verts[tris[i+1]*3];
	//	const float* vc = &verts[tris[i+2]*3];
	//	
	//	int ax = 0, ay = 0;
	//	if (rcAbs(norm[1]) > rcAbs(norm[ax]))
	//		ax = 1;
	//	if (rcAbs(norm[2]) > rcAbs(norm[ax]))
	//		ax = 2;
	//	ax = (1<<ax)&3; // +1 mod 3
	//	ay = (1<<ax)&3; // +1 mod 3
	//	
	//	uva[0] = va[ax]*texScale;
	//	uva[1] = va[ay]*texScale;
	//	uvb[0] = vb[ax]*texScale;
	//	uvb[1] = vb[ay]*texScale;
	//	uvc[0] = vc[ax]*texScale;
	//	uvc[1] = vc[ay]*texScale;
	//	
	//	dd->vertex(va, color, uva);
	//	dd->vertex(vb, color, uvb);
	//	dd->vertex(vc, color, uvc);
	//}
	dd->end();

	dd->texture(false);
}

void duDebugDrawHeightfieldSolid(duDebugDraw* dd, const rcHeightfield& hf)
{
	if (!dd) return;

	//const float* orig = hf.bmin;
	const Vector3& orig = hf.bounds.bmin;
	const float cs = hf.bounds.cs;
	const float ch = hf.bounds.ch;
	
	const int w = hf.width;
	const int h = hf.height;
		
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(255,255,255,255));
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig.x + x*cs;
			float fz = orig.z + y*cs;
			const rcSpan* s = hf.spans[x + y*w];
			while (s)
			{
				duAppendBox(dd, fx, orig.y+s->smin*ch, fz, fx+cs, orig.y + s->smax*ch, fz+cs, fcol);
				s = s->next;
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldWalkable(duDebugDraw* dd, const rcHeightfield& hf)
{
	if (!dd) return;

	const Vector3& orig = hf.bounds.bmin;
	const float cs = hf.bounds.cs;
	const float ch = hf.bounds.ch;
	
	const int w = hf.width;
	const int h = hf.height;
	
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(217,217,217,255));

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig.x + x*cs;
			float fz = orig.z + y*cs;
			const rcSpan* s = hf.spans[x + y*w];
			while (s)
			{
				if (s->area == RC_WALKABLE_AREA)
					fcol[0] = duRGBA(64,128,160,255);
				else if (s->area == RC_NULL_AREA)
					fcol[0] = duRGBA(64,64,64,255);
				else
					fcol[0] = duMultCol(dd->areaToCol(s->area), 200);
				
				duAppendBox(dd, fx, orig.y+s->smin*ch, fz, fx+cs, orig.y + s->smax*ch, fz+cs, fcol);
				s = s->next;
			}
		}
	}
	
	dd->end();
}

void duDebugDrawCompactHeightfieldSolid(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	if (!dd) return;

	const Vector3& orig = chf.bounds.bmin;
	const float cs = chf.bounds.cs;
	const float ch = chf.bounds.ch;

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = orig.x + x*cs;
			const float fz = orig.z + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];

			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				const unsigned char area = chf.areas[i];
				unsigned int color;
				if (area == RC_WALKABLE_AREA)
					color = duRGBA(0,192,255,64);
				else if (area == RC_NULL_AREA)
					color = duRGBA(0,0,0,64);
				else
					color = dd->areaToCol(area);
				
				const float fy = orig.y + (s.y+1)*ch;
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
	if (!dd) return;

	const Vector3& orig = chf.bounds.bmin;
	const float cs = chf.bounds.cs;
	const float ch = chf.bounds.ch;

	dd->begin(DU_DRAW_QUADS);

	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = orig.x + x*cs;
			const float fz = orig.z + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = orig.y + (s.y)*ch;
				unsigned int color;
				if (s.reg)
					color = duIntToCol(s.reg, 192);
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
	if (!dd) return;
	if (!chf.dist) return;
		
	const Vector3& orig = chf.bounds.bmin;
	const float cs = chf.bounds.cs;
	const float ch = chf.bounds.ch;
			
	float maxd = chf.maxDistance;
	if (maxd < 1.0f) maxd = 1;
	const float dscale = 255.0f / maxd;
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = orig.x + x*cs;
			const float fz = orig.z + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = orig.y + (s.y+1)*ch;
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

static void drawLayerPortals(duDebugDraw* dd, const rcHeightfieldLayer* layer)
{
	//const float cs = layer->cs;
	//const float ch = layer->ch;
	auto* bounds = &layer->bounds;
	const float cs = bounds->cs;
	const float ch = bounds->ch;
	const int w = layer->width;
	const int h = layer->height;
	
	unsigned int pcol = duRGBA(255,255,255,255);
	
	const int segs[4*4] = {0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0};
	
	// Layer portals
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const int lh = (int)layer->heights[idx];
			if (lh == 255) continue;
			
			for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
			{
				if (layer->cons[idx] & (1<<(dir+4)))
				{
					const int* seg = &segs[dir*4];
					const float ax = bounds->bmin.x + (x+seg[0])*cs;
					const float ay = bounds->bmin.y + (lh+2)*ch;
					const float az = bounds->bmin.z + (y+seg[1])*cs;
					const float bx = bounds->bmin.x + (x+seg[2])*cs;
					const float by = bounds->bmin.y + (lh+2)*ch;
					const float bz = bounds->bmin.z + (y+seg[3])*cs;
					dd->vertex(ax, ay, az, pcol);
					dd->vertex(bx, by, bz, pcol);
				}
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct rcHeightfieldLayer& layer, const int idx)
{
	//const float cs = layer.cs;
	//const float ch = layer.ch;
	auto& bounds = layer.bounds;
	const float cs = bounds.cs;
	const float ch = bounds.ch;
	const int w = layer.width;
	const int h = layer.height;
	
	unsigned int color = duIntToCol(idx+1, 255);
	
	// Layer bounds
	//float bmin[3], bmax[3];
	//bmin[0] = layer.bmin[0] + layer.minx*cs;
	//bmin[1] = layer.bmin[1];
	//bmin[2] = layer.bmin[2] + layer.miny*cs;
	//bmax[0] = layer.bmin[0] + (layer.maxx+1)*cs;
	//bmax[1] = layer.bmax[1];
	//bmax[2] = layer.bmin[2] + (layer.maxy+1)*cs;
	//duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duTransCol(color,128), 2.0f);
	Vector3 bmin;
	Vector3 bmax;
	bmin.x = bounds.bmin.x + layer.minx*cs;
	bmin.y = bounds.bmin.y;
	bmin.z = bounds.bmin.z + layer.miny*cs;
	bmax.x = bounds.bmin.x + (layer.maxx+1)*cs;
	bmax.y = bounds.bmax.y;
	bmax.z = bounds.bmin.z + (layer.maxy+1)*cs;
	duDebugDrawBoxWire(dd, bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z, duTransCol(color,128), 2.0f);
	
	// Layer height
	dd->begin(DU_DRAW_QUADS);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int lidx = x+y*w;
			const int lh = (int)layer.heights[lidx];
			if (h == 0xff) continue;
			const unsigned char area = layer.areas[lidx];
			
			unsigned int col;
			if (area == RC_WALKABLE_AREA)
				col = duLerpCol(color, duRGBA(0,192,255,64), 32);
			else if (area == RC_NULL_AREA)
				col = duLerpCol(color, duRGBA(0,0,0,64), 32);
			else
				col = duLerpCol(color, dd->areaToCol(area), 32);
			
			const float fx = bounds.bmin.x + x*cs;
			const float fy = bounds.bmin.y + (lh+1)*ch;
			const float fz = bounds.bmin.z + y*cs;
			
			dd->vertex(fx, fy, fz, col);
			dd->vertex(fx, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz, col);
		}
	}
	dd->end();
	
	// Portals
	drawLayerPortals(dd, &layer);
}

void duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset)
{
	if (!dd) return;
	for (int i = 0; i < lset.nlayers; ++i)
		duDebugDrawHeightfieldLayer(dd, lset.layers[i], i);
}

/*
void duDebugDrawLayerContours(duDebugDraw* dd, const struct rcLayerContourSet& lcset)
{
	if (!dd) return;
	
	const float* orig = lcset.bmin;
	const float cs = lcset.cs;
	const float ch = lcset.ch;
	
	const unsigned char a = 255;// (unsigned char)(alpha*255.0f);
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_LINES, 2.0f);
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;

		color = duIntToCol(i, a);

		for (int j = 0; j < c.nverts; ++j)
		{
			const int k = (j+1) % c.nverts;
			const unsigned char* va = &c.verts[j*4];
			const unsigned char* vb = &c.verts[k*4];
			const float ax = orig[0] + va[0]*cs;
			const float ay = orig[1] + (va[1]+1+(i&1))*ch;
			const float az = orig[2] + va[2]*cs;
			const float bx = orig[0] + vb[0]*cs;
			const float by = orig[1] + (vb[1]+1+(i&1))*ch;
			const float bz = orig[2] + vb[2]*cs;
			unsigned int col = color;
			if ((va[3] & 0xf) != 0xf)
			{
				col = duRGBA(255,255,255,128);
				int d = va[3] & 0xf;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
			}
			
			duAppendArrow(dd, ax,ay,az, bx,by,bz, 0.0f, cs*0.5f, col);
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 4.0f);	
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;
		
		for (int j = 0; j < c.nverts; ++j)
		{
			const unsigned char* va = &c.verts[j*4];

			color = duDarkenCol(duIntToCol(i, a));
			if (va[3] & 0x80)
				color = duRGBA(255,0,0,255);

			float fx = orig[0] + va[0]*cs;
			float fy = orig[1] + (va[1]+1+(i&1))*ch;
			float fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, color);
		}
	}
	dd->end();
}

void duDebugDrawLayerPolyMesh(duDebugDraw* dd, const struct rcLayerPolyMesh& lmesh)
{
	if (!dd) return;
	
	const int nvp = lmesh.nvp;
	const float cs = lmesh.cs;
	const float ch = lmesh.ch;
	const float* orig = lmesh.bmin;
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		
		unsigned int color;
		if (lmesh.areas[i] == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (lmesh.areas[i] == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = duIntToCol(lmesh.areas[i], 255);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
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
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
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
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
			{
				const unsigned short* va = &lmesh.verts[vi[0]*3];
				const unsigned short* vb = &lmesh.verts[vi[1]*3];

				const float ax = orig[0] + va[0]*cs;
				const float ay = orig[1] + (va[1]+1+(i&1))*ch;
				const float az = orig[2] + va[2]*cs;
				const float bx = orig[0] + vb[0]*cs;
				const float by = orig[1] + (vb[1]+1+(i&1))*ch;
				const float bz = orig[2] + vb[2]*cs;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				int d = p[nvp+j] & 0xf;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
				
				col = duRGBA(255,255,255,128);
			}
							 
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < lmesh.nverts; ++i)
	{
		const unsigned short* v = &lmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}
*/

static void getContourCenter(const rcContour* cont, const Vector3& orig, float cs, float ch, Vector3& center)
{
	//center[0] = 0;
	//center[1] = 0;
	//center[2] = 0;
	center.clear();

	if (!cont->nverts)
		return;
	for (int i = 0; i < cont->nverts; ++i)
	{
		const int* v = &cont->verts[i*4];
		center.x += (float)v[0];
		center.y += (float)v[1];
		center.z += (float)v[2];
	}
	const float s = 1.0f / cont->nverts;
	center.x *= s * cs;
	center.y *= s * ch;
	center.z *= s * cs;
	center.x += orig.x;
	center.y += orig.y + 4*ch;
	center.z += orig.z;
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

void duDebugDrawRegionConnections(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;
	
	// const float* orig = cset.bmin;
	const Vector3& orig = cset.bounds.bmin;
	const float cs = cset.bounds.cs;
	const float ch = cset.bounds.ch;
	
	// Draw centers
	// float pos[3], pos2[3];
	Vector3 pos, pos2;

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
				duAppendArc(dd, pos.x, pos.y, pos.z, pos2.x, pos2.y, pos2.z, 0.25f, 0.6f, 0.6f, color);
			}
		}
	}
	
	dd->end();

	unsigned char a = (unsigned char)(alpha * 255.0f);

	dd->begin(DU_DRAW_POINTS, 7.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour* cont = &cset.conts[i];
		unsigned int col = duDarkenCol(duIntToCol(cont->reg,a));
		getContourCenter(cont, orig, cs, ch, pos);
		dd->vertex(pos.x, pos.y, pos.z, col);
	}
	dd->end();
}

void duDebugDrawRawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;

	//const float* orig = cset.bmin;
	//const float cs = cset.cs;
	//const float ch = cset.ch;
	const Vector3& orig = cset.bounds.bmin;
	const float cs = cset.bounds.cs;
	const float ch = cset.bounds.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.0f);
			
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duIntToCol(c.reg, a);

		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float fx = orig.x + v[0]*cs;
			float fy = orig.y + (v[1]+1+(i&1))*ch;
			float fz = orig.z + v[2]*cs;
			dd->vertex(fx,fy,fz,color);
			if (j > 0)
				dd->vertex(fx,fy,fz,color);
		}
		// Loop last segment.
		const int* v = &c.rverts[0];
		float fx = orig.x + v[0]*cs;
		float fy = orig.y + (v[1]+1+(i&1))*ch;
		float fz = orig.z + v[2]*cs;
		dd->vertex(fx,fy,fz,color);
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 2.0f);	

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
		
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
			
			float fx = orig.x + v[0]*cs;
			float fy = orig.y + (v[1]+1+(i&1))*ch + off;
			float fz = orig.z + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;

	//const float* orig = cset.bmin;
	//const float cs = cset.cs;
	//const float ch = cset.ch;
	const Vector3& orig = cset.bounds.bmin;
	const float cs = cset.bounds.cs;
	const float ch = cset.bounds.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.5f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		if (!c.nverts)
			continue;
		const unsigned int color = duIntToCol(c.reg, a);
		const unsigned int bcolor = duLerpCol(color,duRGBA(255,255,255,a),128);
		for (int j = 0, k = c.nverts-1; j < c.nverts; k=j++)
		{
			const int* va = &c.verts[k*4];
			const int* vb = &c.verts[j*4];
			unsigned int col = (va[3] & RC_AREA_BORDER) ? bcolor : color; 
			float fx,fy,fz;
			fx = orig.x + va[0]*cs;
			fy = orig.y + (va[1]+1+(i&1))*ch;
			fz = orig.z + va[2]*cs;
			dd->vertex(fx,fy,fz, col);
			fx = orig.x + vb[0]*cs;
			fy = orig.y + (vb[1]+1+(i&1))*ch;
			fz = orig.z + vb[2]*cs;
			dd->vertex(fx,fy,fz, col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
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

			float fx = orig.x + v[0]*cs;
			float fy = orig.y + (v[1]+1+(i&1))*ch + off;
			float fz = orig.z + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawPolyMesh(duDebugDraw* dd, const struct rcPolyMesh& mesh)
{
	if (!dd) return;

	const int nvp = mesh.nvp;
	const float cs = mesh.bounds.cs;
	const float ch = mesh.bounds.ch;
	//const float* orig = mesh.bmin;
	const Vector3& orig = mesh.bounds.bmin;
	
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		const unsigned char area = mesh.areas[i];
		
		unsigned int color;
		if (area == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (area == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = dd->areaToCol(area);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig.x + v[0]*cs;
				const float y = orig.y + (v[1]+1)*ch;
				const float z = orig.z + v[2]*cs;
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
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig.x + v[0]*cs;
				const float y = orig.y + (v[1]+1)*ch + 0.1f;
				const float z = orig.z + v[2]*cs;
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
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
				col = duRGBA(255,255,255,128);
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig.x + v[0]*cs;
				const float y = orig.y + (v[1]+1)*ch + 0.1f;
				const float z = orig.z + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < mesh.nverts; ++i)
	{
		const unsigned short* v = &mesh.verts[i*3];
		const float x = orig.x + v[0]*cs;
		const float y = orig.y + (v[1]+1)*ch + 0.1f;
		const float z = orig.z + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}

void duDebugDrawPolyMeshDetail(duDebugDraw* dd, const struct rcPolyMeshDetail& dmesh)
{
	if (!dd) return;

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
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
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
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
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
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
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const int nverts = (int)m[1];
		const float* verts = &dmesh.verts[bverts*3];
		for (int j = 0; j < nverts; ++j)
			dd->vertex(&verts[j*3], colv);
	}
	dd->end();
}
