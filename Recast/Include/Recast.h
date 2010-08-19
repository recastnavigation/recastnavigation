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
 
#ifndef RECAST_H
#define RECAST_H

#ifdef __GNUC__
#include <stdint.h>
typedef int64_t rcTimeVal;
#else
typedef __int64 rcTimeVal;
#endif

enum rcLogCategory
{
	RC_LOG_PROGRESS = 1,
	RC_LOG_WARNING,
	RC_LOG_ERROR,
};

enum rcBuilTimeLabel
{
	RC_TIME_RASTERIZE_TRIANGLES,
	RC_TIME_BUILD_COMPACTHEIGHFIELD,
	RC_TIME_BUILD_CONTOURS,
	RC_TIME_BUILD_CONTOURS_TRACE,
	RC_TIME_BUILD_CONTOURS_SIMPLIFY,
	RC_TIME_FILTER_BORDER,
	RC_TIME_FILTER_WALKABLE,
	RC_TIME_MEDIAN_AREA,
	RC_TIME_FILTER_LOW_OBSTACLES,
	RC_TIME_BUILD_POLYMESH,
	RC_TIME_MERGE_POLYMESH,
	RC_TIME_ERODE_AREA,
	RC_TIME_MARK_BOX_AREA,
	RC_TIME_MARK_CONVEXPOLY_AREA,
	RC_TIME_BUILD_DISTANCEFIELD,
	RC_TIME_BUILD_DISTANCEFIELD_DIST,
	RC_TIME_BUILD_DISTANCEFIELD_BLUR,
	RC_TIME_BUILD_REGIONS,
	RC_TIME_BUILD_REGIONS_WATERSHED,
	RC_TIME_BUILD_REGIONS_EXPAND,
	RC_TIME_BUILD_REGIONS_FLOOD,
	RC_TIME_BUILD_REGIONS_FILTER,
	RC_TIME_BUILD_POLYMESHDETAIL,
	RC_TIME_MERGE_POLYMESHDETAIL,
	RC_MAX_TIMES
};

// Build context provides several optional utilities needed for the build process,
// such as timing, logging, and build time collecting. 
struct rcBuildContext
{
	// Get current time in platform specific units.
	virtual rcTimeVal getTime() { return 0; }
	// Returns time passed from 'start' to 'end' in microseconds.
	virtual int getDeltaTimeUsec(const rcTimeVal /*start*/, const rcTimeVal /*end*/) { return 0; }

	// Resets log.
	virtual void resetLog() {}
	// Logs a message.
	virtual void log(const rcLogCategory /*category*/, const char* /*format*/, ...) {}

	// Resets build time collecting.
	virtual void resetBuildTimes() {}
	// Reports build time of specified label for accumulation.
	virtual void reportBuildTime(const rcBuilTimeLabel /*label*/, const int /*time*/) {}
	// Returns accumulated build time for specified label, or -1 if no time was reported.
	virtual int getBuildTime(const rcBuilTimeLabel /*label*/) { return -1; }
};


// The units of the parameters are specified in parenthesis as follows:
// (vx) voxels, (wu) world units
struct rcConfig
{
	int width, height;				// Dimensions of the rasterized heighfield (vx)
	int tileSize;					// Width and Height of a tile (vx)
	int borderSize;					// Non-navigable Border around the heightfield (vx)
	float cs, ch;					// Grid cell size and height (wu)
	float bmin[3], bmax[3];			// Grid bounds (wu)
	float walkableSlopeAngle;		// Maximum walkble slope angle in degrees.
	int walkableHeight;				// Minimum height where the agent can still walk (vx)
	int walkableClimb;				// Maximum height between grid cells the agent can climb (vx)
	int walkableRadius;				// Radius of the agent in cells (vx)
	int maxEdgeLen;					// Maximum contour edge length (vx)
	float maxSimplificationError;	// Maximum distance error from contour to cells (vx)
	int minRegionSize;				// Minimum regions size. Smaller regions will be deleted (vx)
	int mergeRegionSize;			// Minimum regions size. Smaller regions will be merged (vx)
	int maxVertsPerPoly;			// Max number of vertices per polygon
	float detailSampleDist;			// Detail mesh sample spacing.
	float detailSampleMaxError;		// Detail mesh simplification max sample error.
};

// Define number of bits in the above structure for smin/smax.
// The max height is used for clamping rasterized values.
static const int RC_SPAN_HEIGHT_BITS = 13;
static const int RC_SPAN_MAX_HEIGHT = (1<<RC_SPAN_HEIGHT_BITS)-1;

// Heightfield span.
struct rcSpan
{
	unsigned int smin : 13;			// Span min height.
	unsigned int smax : 13;			// Span max height.
	unsigned int area : 6;			// Span area type.
	rcSpan* next;					// Next span in column.
};

// Number of spans allocated per pool.
static const int RC_SPANS_PER_POOL = 2048;

// Memory pool used for quick span allocation.
struct rcSpanPool
{
	rcSpanPool* next;					// Pointer to next pool.
	rcSpan items[RC_SPANS_PER_POOL];	// Array of spans.
};

// Dynamic span-heightfield.
struct rcHeightfield
{
	int width, height;			// Dimension of the heightfield.
	float bmin[3], bmax[3];		// Bounding box of the heightfield
	float cs, ch;				// Cell size and height.
	rcSpan** spans;				// Heightfield of spans (width*height).
	rcSpanPool* pools;			// Linked list of span pools.
	rcSpan* freelist;			// Pointer to next free span.
};

struct rcCompactCell
{
	unsigned int index : 24;	// Index to first span in column.
	unsigned int count : 8;		// Number of spans in this column.
};

struct rcCompactSpan
{
	unsigned short y;			// Bottom coordinate of the span.
	unsigned short reg;
	unsigned int con : 24;		// Connections to neighbour cells.
	unsigned int h : 8;			// Height of the span.
};

// Compact static heightfield. 
struct rcCompactHeightfield
{
	rcCompactHeightfield();
	~rcCompactHeightfield();
	
	int width, height;					// Width and height of the heighfield.
	int spanCount;						// Number of spans in the heightfield.
	int walkableHeight, walkableClimb;	// Agent properties.
	unsigned short maxDistance;			// Maximum distance value stored in heightfield.
	unsigned short maxRegions;			// Maximum Region Id stored in heightfield.
	float bmin[3], bmax[3];				// Bounding box of the heightfield.
	float cs, ch;						// Cell size and height.
	rcCompactCell* cells;				// Pointer to width*height cells.
	rcCompactSpan* spans;				// Pointer to spans.
	unsigned short* dist;				// Pointer to per span distance to border.
	unsigned char* areas;				// Pointer to per span area ID.
};

struct rcContour
{
	int* verts;			// Vertex coordinates, each vertex contains 4 components.
	int nverts;			// Number of vertices.
	int* rverts;		// Raw vertex coordinates, each vertex contains 4 components.
	int nrverts;		// Number of raw vertices.
	unsigned short reg;	// Region ID of the contour.
	unsigned char area;	// Area ID of the contour.
};

struct rcContourSet
{
	rcContour* conts;		// Pointer to all contours.
	int nconts;				// Number of contours.
	float bmin[3], bmax[3];	// Bounding box of the heightfield.
	float cs, ch;			// Cell size and height.
};

// Polymesh store a connected mesh of polygons.
// The polygons are store in an array where each polygons takes
// 'nvp*2' elements. The first 'nvp' elements are indices to vertices
// and the second 'nvp' elements are indices to neighbour polygons.
// If a polygona has less than 'bvp' vertices, the remaining indices
// are set to RC_MESH_NULL_IDX. If an polygon edge does not have a neighbour
// the neighbour index is set to RC_MESH_NULL_IDX.
// Vertices can be transformed into world space as follows:
//   x = bmin[0] + verts[i*3+0]*cs;
//   y = bmin[1] + verts[i*3+1]*ch;
//   z = bmin[2] + verts[i*3+2]*cs;
struct rcPolyMesh
{	
	unsigned short* verts;	// Vertices of the mesh, 3 elements per vertex.
	unsigned short* polys;	// Polygons of the mesh, nvp*2 elements per polygon.
	unsigned short* regs;	// Region ID of the polygons.
	unsigned short* flags;	// Per polygon flags.
	unsigned char* areas;	// Area ID of polygons.
	int nverts;				// Number of vertices.
	int npolys;				// Number of polygons.
	int maxpolys;			// Number of allocated polygons.
	int nvp;				// Max number of vertices per polygon.
	float bmin[3], bmax[3];	// Bounding box of the mesh.
	float cs, ch;			// Cell size and height.
};

// Detail mesh generated from a rcPolyMesh.
// Each submesh represents a polygon in the polymesh and they are stored in
// excatly same order. Each submesh is described as 4 values:
// base vertex, vertex count, base triangle, triangle count. That is,
//   const unsigned char* t = &dtl.tris[(tbase+i)*3]; and
//   const float* v = &dtl.verts[(vbase+t[j])*3];
// If the input polygon has 'n' vertices, those vertices are first in the
// submesh vertex list. This allows to compres the mesh by not storing the
// first vertices and using the polymesh vertices instead.

struct rcPolyMeshDetail
{
	unsigned short* meshes;	// Pointer to all mesh data.
	float* verts;			// Pointer to all vertex data.
	unsigned char* tris;	// Pointer to all triangle data.
	int nmeshes;			// Number of meshes.
	int nverts;				// Number of total vertices.
	int ntris;				// Number of triangles.
};

// Allocators and destructors for Recast objects.
rcHeightfield* rcAllocHeightfield();
void rcFreeHeightField(rcHeightfield* hf);

rcCompactHeightfield* rcAllocCompactHeightfield();
void rcFreeCompactHeightfield(rcCompactHeightfield* chf);

rcContourSet* rcAllocContourSet();
void rcFreeContourSet(rcContourSet* cset);

rcPolyMesh* rcAllocPolyMesh();
void rcFreePolyMesh(rcPolyMesh* pmesh);

rcPolyMeshDetail* rcAllocPolyMeshDetail();
void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh);


// If heightfield region ID has the following bit set, the region is on border area
// and excluded from many calculations.
static const unsigned short RC_BORDER_REG = 0x8000;

// If contour region ID has the following bit set, the vertex will be later
// removed in order to match the segments and vertices at tile boundaries.
static const int RC_BORDER_VERTEX = 0x10000;

static const int RC_AREA_BORDER = 0x20000;

enum rcBuildContoursFlags
{
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,	// Tesselate wall edges
	RC_CONTOUR_TESS_AREA_EDGES = 0x02,	// Tesselate edges between areas.
};

// Mask used with contours to extract region id.
static const int RC_CONTOUR_REG_MASK = 0xffff;

// Null index which is used with meshes to mark unset or invalid indices.
static const unsigned short RC_MESH_NULL_IDX = 0xffff;

// Area ID that is considered empty.
static const unsigned char RC_NULL_AREA = 0;

// Area ID that is considered generally walkable.
static const unsigned char RC_WALKABLE_AREA = 63;

// Value returned by rcGetCon() if the direction is not connected.
static const int RC_NOT_CONNECTED = 0x3f;

// Compact span neighbour helpers.
inline void rcSetCon(rcCompactSpan& s, int dir, int i)
{
	const unsigned int shift = (unsigned int)dir*6;
	unsigned int con = s.con;
	s.con = (con & ~(0x3f << shift)) | (((unsigned int)i & 0x3f) << shift);
}

inline int rcGetCon(const rcCompactSpan& s, int dir)
{
	const unsigned int shift = (unsigned int)dir*6;
	return (s.con >> shift) & 0x3f;
}

inline int rcGetDirOffsetX(int dir)
{
	const int offset[4] = { -1, 0, 1, 0, };
	return offset[dir&0x03];
}

inline int rcGetDirOffsetY(int dir)
{
	const int offset[4] = { 0, 1, 0, -1 };
	return offset[dir&0x03];
}

// Common helper functions
template<class T> inline void rcSwap(T& a, T& b) { T t = a; a = b; b = t; }
template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }
template<class T> inline T rcMax(T a, T b) { return a > b ? a : b; }
template<class T> inline T rcAbs(T a) { return a < 0 ? -a : a; }
template<class T> inline T rcSqr(T a) { return a*a; }
template<class T> inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }
float rcSqrt(float x);

// Common vector helper functions.
inline void rcVcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

inline float rcVdot(const float* v1, const float* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

inline void rcVmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0]+v2[0]*s;
	dest[1] = v1[1]+v2[1]*s;
	dest[2] = v1[2]+v2[2]*s;
}

inline void rcVadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]+v2[0];
	dest[1] = v1[1]+v2[1];
	dest[2] = v1[2]+v2[2];
}

inline void rcVsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]-v2[0];
	dest[1] = v1[1]-v2[1];
	dest[2] = v1[2]-v2[2];
}

inline void rcVmin(float* mn, const float* v)
{
	mn[0] = rcMin(mn[0], v[0]);
	mn[1] = rcMin(mn[1], v[1]);
	mn[2] = rcMin(mn[2], v[2]);
}

inline void rcVmax(float* mx, const float* v)
{
	mx[0] = rcMax(mx[0], v[0]);
	mx[1] = rcMax(mx[1], v[1]);
	mx[2] = rcMax(mx[2], v[2]);
}

inline void rcVcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

inline float rcVdist(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx*dx + dy*dy + dz*dz);
}

inline float rcVdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx*dx + dy*dy + dz*dz;
}

inline void rcVnormalize(float* v)
{
	float d = 1.0f / rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

inline bool rcVequal(const float* p0, const float* p1)
{
	static const float thr = rcSqr(1.0f/16384.0f);
	const float d = rcVdistSqr(p0, p1);
	return d < thr;
}

// Calculated bounding box of array of vertices.
// Params:
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	bmin, bmax - (out) bounding box
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax);

// Calculates grid size based on bounding box and grid cell size.
// Params:
//	bmin, bmax - (in) bounding box
//	cs - (in) grid cell size
//	w - (out) grid width
//	h - (out) grid height
void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h);

// Creates and initializes new heightfield.
// Params:
//	hf - (in/out) heightfield to initialize.
//	width - (in) width of the heightfield.
//	height - (in) height of the heightfield.
//	bmin, bmax - (in) bounding box of the heightfield
//	cs - (in) grid cell size
//	ch - (in) grid cell height
bool rcCreateHeightfield(rcBuildContext* ctx, rcHeightfield& hf, int width, int height,
						 const float* bmin, const float* bmax,
						 float cs, float ch);

// Sets the RC_WALKABLE_AREA for every triangle whose slope is below
// the maximun walkable slope angle.
// Params:
//	walkableSlopeAngle - (in) maximun slope angle in degrees.
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	nt - (in) triangle count
//	areas - (out) array of triangle area types
void rcMarkWalkableTriangles(rcBuildContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
							 const int* tris, int nt, unsigned char* areas); 

// Sets the RC_NULL_AREA for every triangle whose slope is steeper than
// the maximun walkable slope angle.
// Params:
//	walkableSlopeAngle - (in) maximun slope angle in degrees.
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	nt - (in) triangle count
//	areas - (out) array of triangle are types
void rcClearUnwalkableTriangles(rcBuildContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
								const int* tris, int nt, unsigned char* areas); 

// Adds span to heighfield.
// The span addition can set to favor flags. If the span is merged to
// another span and the new smax is within 'flagMergeThr' units away
// from the existing span the span flags are merged and stored.
// Params:
//	solid - (in) heighfield where the spans is added to
//  x,y - (in) location on the heighfield where the span is added
//  smin,smax - (in) spans min/max height
//  flags - (in) span flags (zero or WALKABLE)
//  flagMergeThr - (in) merge threshold.
void rcAddSpan(rcBuildContext* ctx, rcHeightfield& solid, const int x, const int y,
			   const unsigned short smin, const unsigned short smax,
			   const unsigned short area, const int flagMergeThr);

// Rasterizes a triangle into heightfield spans.
// Params:
//	v0,v1,v2 - (in) the vertices of the triangle.
//	area - (in) area type of the triangle.
//	solid - (in) heighfield where the triangle is rasterized
//  flagMergeThr - (in) distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangle(rcBuildContext* ctx, const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& solid,
						 const int flagMergeThr = 1);

// Rasterizes indexed triangle mesh into heightfield spans.
// Params:
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	area - (in) array of triangle area types.
//	nt - (in) triangle count
//	solid - (in) heighfield where the triangles are rasterized
//  flagMergeThr - (in) distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangles(rcBuildContext* ctx, const float* verts, const int nv,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes indexed triangle mesh into heightfield spans.
// Params:
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	area - (in) array of triangle area types.
//	nt - (in) triangle count
//	solid - (in) heighfield where the triangles are rasterized
//  flagMergeThr - (in) distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangles(rcBuildContext* ctx, const float* verts, const int nv,
						  const unsigned short* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes the triangles into heightfield spans.
// Params:
//	verts - (in) array of vertices
//	area - (in) array of triangle area types.
//	nt - (in) triangle count
//	solid - (in) heighfield where the triangles are rasterized
void rcRasterizeTriangles(rcBuildContext* ctx, const float* verts, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

// Marks non-walkable low obstacles as walkable if they are closer than walkableClimb
// from a walkable surface. Applying this filter allows to step over low hanging
// low obstacles.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	solid - (in/out) heightfield describing the solid space
// TODO: Missuses ledge flag, must be called before rcFilterLedgeSpans!
void rcFilterLowHangingWalkableObstacles(rcBuildContext* ctx, const int walkableClimb, rcHeightfield& solid);

// Removes WALKABLE flag from all spans that are at ledges. This filtering
// removes possible overestimation of the conservative voxelization so that
// the resulting mesh will not have regions hanging in air over ledges.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	walkableClimb - (in) maximum height between grid cells the agent can climb
//	solid - (in/out) heightfield describing the solid space
void rcFilterLedgeSpans(rcBuildContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid);

// Removes WALKABLE flag from all spans which have smaller than
// 'walkableHeight' clearane above them.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	solid - (in/out) heightfield describing the solid space
void rcFilterWalkableLowHeightSpans(rcBuildContext* ctx, int walkableHeight, rcHeightfield& solid);

// Returns number of spans contained in a heightfield.
// Params:
//	hf - (in) heightfield to be compacted
// Returns number of spans.
int rcGetHeightFieldSpanCount(rcBuildContext* ctx, rcHeightfield& hf);

// Builds compact representation of the heightfield.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	walkableClimb - (in) maximum height between grid cells the agent can climb
//  flags - (in) require flags for a cell to be included in the compact heightfield.
//	hf - (in) heightfield to be compacted
//	chf - (out) compact heightfield representing the open space.
// Returns false if operation ran out of memory.
bool rcBuildCompactHeightfield(rcBuildContext* ctx, const int walkableHeight, const int walkableClimb,
							   rcHeightfield& hf, rcCompactHeightfield& chf);

// Erodes walkable area.
// Params:
//  radius - (in) radius of erosion (max 255).
//	chf - (in/out) compact heightfield to erode.
// Returns false if operation ran out of memory.
bool rcErodeWalkableArea(rcBuildContext* ctx, int radius, rcCompactHeightfield& chf);

// Applies median filter to walkable area types, removing noise.
// Params:
//	chf - (in/out) compact heightfield to erode.
// Returns false if operation ran out of memory.
bool rcMedianFilterWalkableArea(rcBuildContext* ctx, rcCompactHeightfield& chf);

// Marks the area of the convex polygon into the area type of the compact heighfield.
// Params:
//  bmin/bmax - (in) bounds of the axis aligned box.
//  areaId - (in) area ID to mark.
//	chf - (in/out) compact heightfield to mark.
void rcMarkBoxArea(rcBuildContext* ctx, const float* bmin, const float* bmax, unsigned char areaId,
				   rcCompactHeightfield& chf);

// Marks the area of the convex polygon into the area type of the compact heighfield.
// Params:
//  verts - (in) vertices of the convex polygon.
//  nverts - (in) number of vertices in the polygon.
//  hmin/hmax - (in) min and max height of the polygon.
//  areaId - (in) area ID to mark.
//	chf - (in/out) compact heightfield to mark.
void rcMarkConvexPolyArea(rcBuildContext* ctx, const float* verts, const int nverts,
						  const float hmin, const float hmax, unsigned char areaId,
						  rcCompactHeightfield& chf);

// Builds distance field and stores it into the combat heightfield.
// Params:
//	chf - (in/out) compact heightfield representing the open space.
// Returns false if operation ran out of memory.
bool rcBuildDistanceField(rcBuildContext* ctx, rcCompactHeightfield& chf);

// Divides the walkable heighfied into simple regions using watershed partitioning.
// Each region has only one contour and no overlaps.
// The regions are stored in the compact heightfield 'reg' field.
// The regions will be shrinked by the radius of the agent.
// The process sometimes creates small regions. The parameter
// 'minRegionSize' specifies the smallest allowed regions size.
// If the area of a regions is smaller than allowed, the regions is
// removed or merged to neighbour region. 
// Params:
//	chf - (in/out) compact heightfield representing the open space.
//	minRegionSize - (in) the smallest allowed regions size.
//	maxMergeRegionSize - (in) the largest allowed regions size which can be merged.
// Returns false if operation ran out of memory.
bool rcBuildRegions(rcBuildContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionSize, const int mergeRegionSize);

// Divides the walkable heighfied into simple regions using simple monotone partitioning.
// Each region has only one contour and no overlaps.
// The regions are stored in the compact heightfield 'reg' field.
// The regions will be shrinked by the radius of the agent.
// The process sometimes creates small regions. The parameter
// 'minRegionSize' specifies the smallest allowed regions size.
// If the area of a regions is smaller than allowed, the regions is
// removed or merged to neighbour region. 
// Params:
//	chf - (in/out) compact heightfield representing the open space.
//	minRegionSize - (in) the smallest allowed regions size.
//	maxMergeRegionSize - (in) the largest allowed regions size which can be merged.
// Returns false if operation ran out of memory.
bool rcBuildRegionsMonotone(rcBuildContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionSize, const int mergeRegionSize);

// Builds simplified contours from the regions outlines.
// Params:
//	chf - (in) compact heightfield which has regions set.
//	maxError - (in) maximum allowed distance between simplified countour and cells.
//	maxEdgeLen - (in) maximum allowed contour edge length in cells.
//	cset - (out) Resulting contour set.
//	flags - (in) build flags, see rcBuildContoursFlags.
// Returns false if operation ran out of memory.
bool rcBuildContours(rcBuildContext* ctx, rcCompactHeightfield& chf,
					 const float maxError, const int maxEdgeLen,
					 rcContourSet& cset, const int flags = RC_CONTOUR_TESS_WALL_EDGES);

// Builds connected convex polygon mesh from contour polygons.
// Params:
//	cset - (in) contour set.
//	nvp - (in) maximum number of vertices per polygon.
//	mesh - (out) poly mesh.
// Returns false if operation ran out of memory.
bool rcBuildPolyMesh(rcBuildContext* ctx, rcContourSet& cset, int nvp, rcPolyMesh& mesh);

bool rcMergePolyMeshes(rcBuildContext* ctx, rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh);

// Builds detail triangle mesh for each polygon in the poly mesh.
// Params:
//	mesh - (in) poly mesh to detail.
//	chf - (in) compacy height field, used to query height for new vertices.
//  sampleDist - (in) spacing between height samples used to generate more detail into mesh.
//  sampleMaxError - (in) maximum allowed distance between simplified detail mesh and height sample.
//	pmdtl - (out) detail mesh.
// Returns false if operation ran out of memory.
bool rcBuildPolyMeshDetail(rcBuildContext* ctx, const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
						   const float sampleDist, const float sampleMaxError,
						   rcPolyMeshDetail& dmesh);

bool rcMergePolyMeshDetails(rcBuildContext* ctx, rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh);


#endif // RECAST_H
