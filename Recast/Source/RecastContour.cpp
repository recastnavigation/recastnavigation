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
#include <string.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastLog.h"
#include "RecastTimer.h"


static int getCornerHeight(int x, int y, int i, int dir,
						   const rcCompactHeightfield& chf)
{
	const rcCompactSpan& s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir+1) & 0x3;
	if (rcGetCon(s, dir) != 0xf)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		if (rcGetCon(as, dirp) != 0xf)
		{
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dirp);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
		}
	}
	if (rcGetCon(s, dirp) != 0xf)
	{
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		if (rcGetCon(as, dir) != 0xf)
		{
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
		}
	}
	
	return ch;
}

static void walkContour(int x, int y, int i,
						rcCompactHeightfield& chf,
						unsigned char* flags, rcIntArray& points)
{
	// Choose the first non-connected edge
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0)
		dir++;
	
	unsigned char startDir = dir;
	int starti = i;
	
	int iter = 0;
	while (++iter < 40000)
	{
		if (flags[i] & (1 << dir))
		{
			// Choose the edge corner
			int px = x;
			int py = getCornerHeight(x, y, i, dir, chf);
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			int r = 0;
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != 0xf)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				const rcCompactSpan& as = chf.spans[ai];
				r = (int)as.reg;
			}
			
			points.push(px);
			points.push(py);
			points.push(pz);
			points.push(r);
			
			flags[i] &= ~(1 << dir); // Remove visited edges
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != 0xf)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}
}

static float distancePtSeg(int x, int y, int z,
						   int px, int py, int pz,
						   int qx, int qy, int qz)
{
/*	float pqx = (float)(qx - px);
	float pqy = (float)(qy - py);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dy = (float)(y - py);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqy*pqy + pqz*pqz;
	float t = pqx*dx + pqy*dy + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dy = py + t*pqy - y;
	dz = pz + t*pqz - z;
	
	return dx*dx + dy*dy + dz*dz;*/

	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	return dx*dx + dz*dz;
}

static void simplifyContour(rcIntArray& points, rcIntArray& simplified, float maxError, int maxEdgeLen)
{
	// Add initial points.
	bool noConnections = true;
	for (int i = 0; i < points.size(); i += 4)
	{
		if (points[i+3] != 0)
		{
			noConnections = false;
			break;
		}
	}
	
	if (noConnections)
	{
		// If there is no connections at all,
		// create some initial points for the simplification process. 
		// Find lower-left and upper-right vertices of the contour.
		int llx = points[0];
		int lly = points[1];
		int llz = points[2];
		int lli = 0;
		int urx = points[0];
		int ury = points[1];
		int urz = points[2];
		int uri = 0;
		for (int i = 0; i < points.size(); i += 4)
		{
			int x = points[i+0];
			int y = points[i+1];
			int z = points[i+2];
			if (x < llx || (x == llx && z < llz))
			{
				llx = x;
				lly = y;
				llz = z;
				lli = i/4;
			}
			if (x >= urx || (x == urx && z > urz))
			{
				urx = x;
				ury = y;
				urz = z;
				uri = i/4;
			}
		}
		simplified.push(llx);
		simplified.push(lly);
		simplified.push(llz);
		simplified.push(lli);
		
		simplified.push(urx);
		simplified.push(ury);
		simplified.push(urz);
		simplified.push(uri);
	}
	else
	{
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		for (int i = 0, ni = points.size()/4; i < ni; ++i)
		{
			int ii = (i+1) % ni;
			if (points[i*4+3] != points[ii*4+3])
			{
				simplified.push(points[i*4+0]);
				simplified.push(points[i*4+1]);
				simplified.push(points[i*4+2]);
				simplified.push(i);
			}
		}	
	}
	
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	const int pn = points.size()/4;
	for (int i = 0; i < simplified.size()/4; )
	{
		int ii = (i+1) % (simplified.size()/4);
		
		int ax = simplified[i*4+0];
		int ay = simplified[i*4+1];
		int az = simplified[i*4+2];
		int ai = simplified[i*4+3];
		
		int bx = simplified[ii*4+0];
		int by = simplified[ii*4+1];
		int bz = simplified[ii*4+2];
		int bi = simplified[ii*4+3];
		
		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci = (ai+1) % pn;
		
		// Tesselate only outer edges.
		if (points[ci*4+3] == 0)
		{
			while (ci != bi)
			{
				float d = distancePtSeg(points[ci*4+0], points[ci*4+1]/4, points[ci*4+2],
										ax, ay/4, az, bx, by/4, bz);
				if (d > maxd)
				{
					maxd = d;
					maxi = ci;
				}
				ci = (ci+1) % pn;
			}
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			// Add space for the new point.
			simplified.resize(simplified.size()+4);
			int n = simplified.size()/4;
			for (int j = n-1; j > i; --j)
			{
				simplified[j*4+0] = simplified[(j-1)*4+0];
				simplified[j*4+1] = simplified[(j-1)*4+1];
				simplified[j*4+2] = simplified[(j-1)*4+2];
				simplified[j*4+3] = simplified[(j-1)*4+3];
			}
			// Add the point.
			simplified[(i+1)*4+0] = points[maxi*4+0];
			simplified[(i+1)*4+1] = points[maxi*4+1];
			simplified[(i+1)*4+2] = points[maxi*4+2];
			simplified[(i+1)*4+3] = maxi;
		}
		else
		{
			++i;
		}
	}
	
	// Split too long edges.
	if (maxEdgeLen > 0)
	{
		for (int i = 0; i < simplified.size()/4; )
		{
			int ii = (i+1) % (simplified.size()/4);
			
			int ax = simplified[i*4+0];
			int az = simplified[i*4+2];
			int ai = simplified[i*4+3];
			
			int bx = simplified[ii*4+0];
			int bz = simplified[ii*4+2];
			int bi = simplified[ii*4+3];
			
			// Find maximum deviation from the segment.
			int maxi = -1;
			int ci = (ai+1) % pn;
			
			// Tesselate only outer edges.
			if (points[ci*4+3] == 0)
			{
				int dx = bx - ax;
				int dz = bz - az;
				if (dx*dx + dz*dz > maxEdgeLen*maxEdgeLen)
				{
					int n = bi < ai ? (bi+pn - ai) : (bi - ai);
					maxi = (ai + n/2) % pn;
				}
			}
			
			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if (maxi != -1)
			{
				// Add space for the new point.
				simplified.resize(simplified.size()+4);
				int n = simplified.size()/4;
				for (int j = n-1; j > i; --j)
				{
					simplified[j*4+0] = simplified[(j-1)*4+0];
					simplified[j*4+1] = simplified[(j-1)*4+1];
					simplified[j*4+2] = simplified[(j-1)*4+2];
					simplified[j*4+3] = simplified[(j-1)*4+3];
				}
				// Add the point.
				simplified[(i+1)*4+0] = points[maxi*4+0];
				simplified[(i+1)*4+1] = points[maxi*4+1];
				simplified[(i+1)*4+2] = points[maxi*4+2];
				simplified[(i+1)*4+3] = maxi;
			}
			else
			{
				++i;
			}
		}
	}
	
	for (int i = 0; i < simplified.size()/4; ++i)
	{
		int ai = (simplified[i*4+3]+1) % pn;
		simplified[i*4+3] = points[ai*4+3];
	}
	
}

static void removeDegenerateSegments(rcIntArray& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	for (int i = 0; i < simplified.size()/4; ++i)
	{
		int ni = i+1;
		if (ni >= (simplified.size()/4))
			ni = 0;
			
		if (simplified[i*4+0] == simplified[ni*4+0] &&
			simplified[i*4+2] == simplified[ni*4+2])
		{
			// Degenerate segment, remove.
			for (int j = i; j < simplified.size()/4-1; ++j)
			{
				simplified[j*4+0] = simplified[(j+1)*4+0];
				simplified[j*4+1] = simplified[(j+1)*4+1];
				simplified[j*4+2] = simplified[(j+1)*4+2];
				simplified[j*4+3] = simplified[(j+1)*4+3];
			}
			simplified.pop();
		}
	}
}

static int calcAreaOfPolygon2D(const int* verts, const int nverts)
{
	int area = 0;
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		const int* vi = &verts[i*4];
		const int* vj = &verts[j*4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area+1) / 2;
}

static void getClosestIndices(const int* vertsa, const int nvertsa,
							  const int* vertsb, const int nvertsb,
							  int& ia, int& ib)
{
	int closestDist = 0xfffffff;
	for (int i = 0; i < nvertsa; ++i)
	{
		const int* va = &vertsa[i*4];
		for (int j = 0; j < nvertsb; ++j)
		{
			const int* vb = &vertsb[j*4];
			const int dx = vb[0] - va[0];
			const int dz = vb[2] - va[2];
			const int d = dx*dx + dz*dz;
			if (d < closestDist)
			{
				ia = i;
				ib = j;
				closestDist = d;
			}
		}
	}
}

static bool mergeContours(rcContour& ca, rcContour& cb, int ia, int ib)
{
	const int maxVerts = ca.nverts + cb.nverts + 2;
	int* verts = new int[maxVerts*4];
	if (!verts)
		return false;

	int nv = 0;

	// Copy contour A.
	for (int i = 0; i <= ca.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &ca.verts[((ia+i)%ca.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}

	// Copy contour B
	for (int i = 0; i <= cb.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &cb.verts[((ib+i)%cb.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}
	
	delete [] ca.verts;
	ca.verts = verts;
	ca.nverts = nv;

	delete [] cb.verts;
	cb.verts = 0;
	cb.nverts = 0;
	
	return true;
}

bool rcBuildContours(rcCompactHeightfield& chf,
					 float maxError, int maxEdgeLen,
					 rcContourSet& cset)
{
	const int w = chf.width;
	const int h = chf.height;
	
	rcTimeVal startTime = rcGetPerformanceTimer();
	
	const int maxContours = chf.maxRegions*2;
	cset.conts = new rcContour[maxContours];
	if (!cset.conts)
		return false;
	cset.nconts = 0;
	
	unsigned char* flags = new unsigned char[chf.spanCount];
	if (!flags)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags'.");
		return false;
	}
	
	rcTimeVal traceStartTime = rcGetPerformanceTimer();
	
	// Mark boundaries.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = chf.spans[i];
				if (!s.reg || (s.reg & 0x8000))
				{
					flags[i] = 0;
					continue;
				}
				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r = 0;
					if (rcGetCon(s, dir) != 0xf)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const rcCompactSpan& as = chf.spans[ai];
						r = as.reg;
					}
					if (r == s.reg)
						res |= (1 << dir);
				}
				flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
			}
		}
	}
	
	rcTimeVal traceEndTime = rcGetPerformanceTimer();
	
	rcTimeVal simplifyStartTime = rcGetPerformanceTimer();
	
	rcIntArray verts(256);
	rcIntArray simplified(64);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}
				unsigned short reg = chf.spans[i].reg;
				if (!reg || (reg & 0x8000))
					continue;
				
				verts.resize(0);
				simplified.resize(0);
				walkContour(x, y, i, chf, flags, verts);
				simplifyContour(verts, simplified, maxError, maxEdgeLen);
				removeDegenerateSegments(simplified);
				
				// Store region->contour remap info.
				// Create contour.
				if (simplified.size()/4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						if (rcGetLog())
							rcGetLog()->log(RC_LOG_ERROR, "rcBuildContours: Too many contours %d, max %d.", cset.nconts, maxContours);
						return false;
					}
						
					rcContour* cont = &cset.conts[cset.nconts++];
					
					cont->nverts = simplified.size()/4;
					cont->verts = new int[cont->nverts*4];
					memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts*4);
					
					cont->nrverts = verts.size()/4;
					cont->rverts = new int[cont->nrverts*4];
					memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts*4);
					
/*					cont->cx = cont->cy = cont->cz = 0;
					for (int i = 0; i < cont->nverts; ++i)
					{
						cont->cx += cont->verts[i*4+0];
						cont->cy += cont->verts[i*4+1];
						cont->cz += cont->verts[i*4+2];
					}
					cont->cx /= cont->nverts;
					cont->cy /= cont->nverts;
					cont->cz /= cont->nverts;*/
					
					cont->reg = reg;
				}
			}
		}
	}
	
	// Check and merge droppings.
	// Sometimes the previous algorithms can fail and create several countours
	// per area. This pass will try to merge the holes into the main region.
	for (int i = 0; i < cset.nconts; ++i)
	{
		rcContour& cont = cset.conts[i];
		// Check if the contour is would backwards.
		if (calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0)
		{
			// Find another contour which has the same region ID.
			int mergeIdx = -1;
			for (int j = 0; j < cset.nconts; ++j)
			{
				if (i == j) continue;
				if (cset.conts[j].nverts && cset.conts[j].reg == cont.reg)
				{
					// Make sure the polygon is correctly oriented.
					if (calcAreaOfPolygon2D(cset.conts[j].verts, cset.conts[j].nverts))
					{
						mergeIdx = j;
						break;
					}
				}
			}
			if (mergeIdx == -1)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_WARNING, "rcBuildContours: Could not find merge target for bad contour %d.", i);
			}
			else
			{
				rcContour& mcont = cset.conts[mergeIdx];
				// Merge by closest points.
				int ia, ib;
				getClosestIndices(mcont.verts, mcont.nverts, cont.verts, cont.nverts, ia, ib);
				if (!mergeContours(mcont, cont, ia, ib))
				{
					if (rcGetLog())
						rcGetLog()->log(RC_LOG_WARNING, "rcBuildContours: Failed to merge contours %d and %d.", i, mergeIdx);
				}
			}
		}
	}
	
		
	delete [] flags;
	
	rcTimeVal simplifyEndTime = rcGetPerformanceTimer();
	
	rcTimeVal endTime = rcGetPerformanceTimer();
	
//	if (rcGetLog())
//	{
//		rcGetLog()->log(RC_LOG_PROGRESS, "Create contours: %.3f ms", rcGetDeltaTimeUsec(startTime, endTime)/1000.0f);
//		rcGetLog()->log(RC_LOG_PROGRESS, " - boundary: %.3f ms", rcGetDeltaTimeUsec(boundaryStartTime, boundaryEndTime)/1000.0f);
//		rcGetLog()->log(RC_LOG_PROGRESS, " - contour: %.3f ms", rcGetDeltaTimeUsec(contourStartTime, contourEndTime)/1000.0f);
//	}

	if (rcGetBuildTimes())
	{
		rcGetBuildTimes()->buildContours += rcGetDeltaTimeUsec(startTime, endTime);
		rcGetBuildTimes()->buildContoursTrace += rcGetDeltaTimeUsec(traceStartTime, traceEndTime);
		rcGetBuildTimes()->buildContoursSimplify += rcGetDeltaTimeUsec(simplifyStartTime, simplifyEndTime);
	}
	
	return true;
}

static bool insertPoint(rcContour* c, int idx, const int* v)
{
	int* newVerts = new int[(c->nverts+1)*4];
	if (!newVerts)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "insertPoint: Out of memory 'newVerts'.");
		return false;
	}
	
	if (idx > 0)
		memcpy(newVerts, c->verts, sizeof(int)*4*idx);
	
	newVerts[idx*4+0] = v[0];
	newVerts[idx*4+1] = v[1];
	newVerts[idx*4+2] = v[2];
	newVerts[idx*4+3] = 0;
	
	if (c->nverts - idx > 0)
		memcpy(&newVerts[(idx+1)*4], &c->verts[idx*4], sizeof(int)*4*(c->nverts - idx));
	
	delete [] c->verts;
	
	c->verts = newVerts;
	c->nverts++;
	
	return true;
}

static void calcBox(const int* v0, const int* v1, int* bounds)
{
	bounds[0] = rcMin(v0[0], v1[0]);
	bounds[1] = rcMin(v0[1], v1[1]);
	bounds[2] = rcMin(v0[2], v1[2]);
	bounds[3] = rcMax(v0[0], v1[0]);
	bounds[4] = rcMax(v0[1], v1[1]);
	bounds[5] = rcMax(v0[2], v1[2]);
}

/*inline bool checkOverlapBoxY(const int* a, const int* b)
{
	bool overlap = true;
	overlap = (a[0] >= b[3+0] || a[3+0] <= b[0]) ? false : overlap;
	overlap = (a[1] >= b[3+1] || a[3+1] <= b[1]) ? false : overlap;
	overlap = (a[2] >= b[3+2] || a[3+2] <= b[2]) ? false : overlap;
	return overlap;
}*/

inline bool checkOverlapBoxY(const int* a, const int* b)
{
	bool overlap = true;
	overlap = (a[1] > b[3+1] || a[3+1] < b[1]) ? false : overlap;
	return overlap;
}

static bool conformVertex(rcContourSet* cset, const int* v,
						  const int pminy, const int pmaxy,
						  const int nminy, const int nmaxy,
						  const int walkableClimb)
{
	for (int i = 0; i < cset->nconts; ++i)
	{
		rcContour* c = &cset->conts[i];
		for (int j = 0; j < c->nverts; ++j)
		{
			const int k = (j+1) % c->nverts;
			const int* vj = &c->verts[j*4];
			const int* vk = &c->verts[k*4];

			const int miny = rcMin(vj[1], vk[1]); // - (walkableClimb-1);
			const int maxy = rcMax(vj[1], vk[1]); // + (walkableClimb-1);

			// Is edge within y-range.
			if ((miny > pmaxy || maxy < pminy) &&
				(miny > nmaxy || maxy < nminy))
				continue;

			if (vj[0] == vk[0] && vj[0] == v[0])
			{
				// The segment is x edge.
				const int minz = rcMin(vj[2], vk[2]);
				const int maxz = rcMax(vj[2], vk[2]);
				if (v[2] > minz && v[2] < maxz)
				{
					return insertPoint(c, j+1, v);
				}
			}
			else if (vj[2] == vk[2] && vj[2] == v[2])
			{
				// The segment is z edge.
				const int minx = rcMin(vj[0], vk[0]);
				const int maxx = rcMax(vj[0], vk[0]);
				if (v[0] > minx && v[0] < maxx)
				{
					return insertPoint(c, j+1, v);
				}
			}
		}
	}
	return true;
}		

bool rcFixupAdjacentContours(rcContourSet* cseta, rcContourSet* csetb,
							 const int walkableClimb, const int edgex, const int edgez)
{
	if (!cseta || !csetb)
		return true;

	rcTimeVal startTime = rcGetPerformanceTimer();

//	int nbox[6], pbox[6];

	for (int i = 0; i < cseta->nconts; ++i)
	{
		const rcContour& c = cseta->conts[i];
		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			const int* pv = &c.verts[((j+c.nverts-1)%c.nverts)*4];
			const int* nv = &c.verts[((j+1)%c.nverts)*4];

//			if (v[0] == edgex || v[2] == edgez)
			{				
				const int pminy = rcMin(v[1], pv[1]);
				const int pmaxy = rcMax(v[1], pv[1]);
				const int nminy = rcMin(v[1], nv[1]);
				const int nmaxy = rcMax(v[1], nv[1]);
				
				if (!conformVertex(csetb, v, pminy, pmaxy, nminy, nmaxy, walkableClimb))
					return false;
			}
		}
	}

	for (int i = 0; i < csetb->nconts; ++i)
	{
		const rcContour& c = csetb->conts[i];
		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			const int* pv = &c.verts[((j+c.nverts-1)%c.nverts)*4];
			const int* nv = &c.verts[((j+1)%c.nverts)*4];
			
			// If the vertex is at the tile edge, make sure it also exists in
			// the neighbour contour set.
//			if (v[0] == edgex || v[2] == edgez)
			{
				const int pminy = rcMin(v[1], pv[1]);
				const int pmaxy = rcMax(v[1], pv[1]);
				const int nminy = rcMin(v[1], nv[1]);
				const int nmaxy = rcMax(v[1], nv[1]);
				
				if (!conformVertex(cseta, v, pminy, pmaxy, nminy, nmaxy, walkableClimb))
					return false;
			}
		}
	}
	
	rcTimeVal endTime = rcGetPerformanceTimer();
	if (rcGetBuildTimes())
		rcGetBuildTimes()->fixupContours += rcGetDeltaTimeUsec(startTime, endTime);
		
	return true;
}

void rcTranslateContours(rcContourSet* cset, int dx, int dy, int dz)
{
	if (!cset) return;
	
	for (int i = 0; i < cset->nconts; ++i)
	{
		rcContour& cont = cset->conts[i];
		for (int i = 0; i < cont.nverts; ++i)
		{
			int* v = &cont.verts[i*4];
			v[0] += dx;
			v[1] += dy;
			v[2] += dz;
		}
		for (int i = 0; i < cont.nrverts; ++i)
		{
			int* v = &cont.rverts[i*4];
			v[0] += dx;
			v[1] += dy;
			v[2] += dz;
		}
	}
}

