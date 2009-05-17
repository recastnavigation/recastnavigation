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
 
#ifndef RECAST_H
#define RECAST_H

struct rcConfig
{
	int width, height;				// Dimensions of the rasterized heighfield
	int tileSize;					// Size if a tile.
	int borderSize;					// Non-navigable Border around the heightfield.
	float cs, ch;					// Grid cell size and height.
	float bmin[3], bmax[3];			// Grid bounds.
	float walkableSlopeAngle;		// Maximum walkble slope angle in degrees.
	int walkableHeight;				// Minimum height where the agent can still walk.
	int walkableClimb;				// Maximum height between grid cells the agent can climb. 
	int walkableRadius;				// Radius of the agent in cells.
	int maxEdgeLen;					// Maximum contour edge length in cells.
	float maxSimplificationError;	// Maximum distance error from contour to cells.
	int minRegionSize;				// Minimum regions size. Smaller regions will be deleted.
	int mergeRegionSize;			// Minimum regions size. Smaller regions will be merged.
	int maxVertsPerPoly;			// Max number of vertices per polygon.
};

struct rcSpan
{
	unsigned int smin : 15;			// Span min height.
	unsigned int smax : 15;			// Span max height.
	unsigned int flags : 2;			// Span flags.
	rcSpan* next;
};

static const int RC_SPANS_PER_POOL = 2048; 

struct rcSpanPool
{
	rcSpanPool* next;
	rcSpan items[1];
};

struct rcHeightfield
{
	inline rcHeightfield() : width(0), height(0), spans(0), pools(0), freelist(0) {}
	inline ~rcHeightfield()
	{
		delete [] spans;
		while (pools)
		{
			rcSpanPool* next = pools->next;
			delete [] reinterpret_cast<unsigned char*>(pools);
			pools = next;
		}
	}
	int width, height;
	rcSpan** spans;
	rcSpanPool* pools;
	rcSpan* freelist;
};

struct rcCompactCell
{
	unsigned int index : 24;
	unsigned int count : 8;
};

struct rcCompactSpan
{
	unsigned short y;
	unsigned short reg;
	unsigned short dist;
	unsigned short con;
	unsigned char h;
	unsigned char flags;
};

struct rcCompactHeightfield
{
	inline rcCompactHeightfield() : cells(0), spans(0), maxDistance(0), maxRegions(0) {}
	inline ~rcCompactHeightfield() { delete [] cells; delete [] spans; }
	int width, height;
	int spanCount;
	int walkableHeight, walkableClimb;
	unsigned short maxDistance; 
	unsigned short maxRegions;
	float bmin[3], bmax[3];
	float cs, ch;
	rcCompactCell* cells;
	rcCompactSpan* spans;
};

struct rcContour
{
	inline rcContour() : verts(0), nverts(0), rverts(0), nrverts(0) { }
	inline ~rcContour() { delete [] verts; delete [] rverts; }
	int* verts;
	int nverts;
	int* rverts;
	int nrverts;
	unsigned short reg;
};

struct rcContourSet
{
	inline rcContourSet() : conts(0), nconts(0) {}
	inline ~rcContourSet() { delete [] conts; }
	rcContour* conts;
	int nconts;
};

struct rcPolyMesh
{
	inline rcPolyMesh() : verts(0), polys(0), nverts(0), npolys(0), nvp(3) {}
	inline ~rcPolyMesh() { delete [] verts; delete [] polys; }
	unsigned short* verts;
	unsigned short* polys;
	int nverts;
	int npolys;
	int nvp;
	float bmin[3], bmax[3];
	float cs, ch;
};

class rcIntArray
{
	int* m_data;
	int m_size, m_cap;
public:
	inline rcIntArray() : m_data(0), m_size(0), m_cap(0) {}
	inline rcIntArray(int n) : m_data(0), m_size(0), m_cap(n) { m_data = new int[n]; }
	inline ~rcIntArray() { delete [] m_data; }
	void resize(int n);
	inline void push(int item) { resize(m_size+1); m_data[m_size-1] = item; }
	inline int pop() { if (m_size > 0) m_size--; return m_data[m_size]; }
	inline const int& operator[](int i) const { return m_data[i]; }
	inline int& operator[](int i) { return m_data[i]; }
	inline int size() const { return m_size; }
};

enum rcSpanFlags
{
	RC_WALKABLE = 0x01,
	RC_REACHABLE = 0x02,
};

// Comppact span neighbour helpers.
inline int rcGetCon(const rcCompactSpan& s, int dir)
{
	return (s.con >> (dir*4)) & 0xf;
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

// Common vector helper functions.
inline void vcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]; 
}

inline float vdot(const float* v1, const float* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

inline void vsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]-v2[0];
	dest[1] = v1[1]-v2[1];
	dest[2] = v1[2]-v2[2];
}

inline void vmin(float* mn, const float* v)
{
	mn[0] = rcMin(mn[0], v[0]);
	mn[1] = rcMin(mn[1], v[1]);
	mn[2] = rcMin(mn[2], v[2]);
}

inline void vmax(float* mx, const float* v)
{
	mx[0] = rcMax(mx[0], v[0]);
	mx[1] = rcMax(mx[1], v[1]);
	mx[2] = rcMax(mx[2], v[2]);
}

inline void vcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

inline float vdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx*dx + dy*dy + dz*dz;
}

inline void vnormalize(float* v)
{
	float d = 1.0f / sqrtf(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

inline bool vequal(const float* p0, const float* p1)
{
	static const float thr = rcSqr(1.0f/16384.0f);
	const float d = vdistSqr(p0, p1);
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
bool rcCreateHeightfield(rcHeightfield& hf, int width, int height);

// Sets the WALKABLE flag for every triangle whose slope is below
// the maximun walkable slope angle.
// Params:
//	walkableSlopeAngle - (in) maximun slope angle in degrees.
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	nt - (in) triangle count
//	flags - (out) array of triangle flags
void rcMarkWalkableTriangles(const float walkableSlopeAngle,
							 const float* verts, int nv,
							 const int* tris, int nt,
							 unsigned char* flags); 

// Rasterizes the triangles into heightfield spans.
// Params:
//	bmin, bmax - (in) bounding box of the heightfield
//	cs - (in) grid cell size
//	ch - (in) grid cell height
//	verts - (in) array of vertices
//	nv - (in) vertex count
//	tris - (in) array of triangle vertex indices
//	norms - (in) array of triangle normals
//	flags - (in) array of triangle flags (uses WALKABLE)
//	nt - (in) triangle count
//	solid - (in) heighfield where the triangles are rasterized
void rcRasterizeTriangles(const float* bmin, const float* bmax,
						  float cs, float ch,
						  const float* verts, int nv,
						  const int* tris, const unsigned char* flags, int nt,
						  rcHeightfield& solid);

void rcFilterWalkableBorderSpans(const int walkableHeight,
								 const int walkableClimb,
								 rcHeightfield& solid);

// Removes WALKABLE flag from all spans which have smaller than
// 'walkableHeight' clearane above them.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	solid - (in/out) heightfield describing the solid space
void rcFilterWalkableLowHeightSpans(int walkableHeight,
									rcHeightfield& solid);

// Marks spans which are reachable from any of the topmost spans.
// Params:
//	walkableHeight - (in) minimum height where the agent can still walk
//	walkableClimb - (in) maximum height between grid cells the agent can climb
//	solid - (in/out) heightfield describing the solid space
// Returns false if operation ran out of memory.
bool rcMarkReachableSpans(const int walkableHeight,
						  const int walkableClimb,
						  rcHeightfield& solid);

// Builds compact representation of the heightfield.
// Params:
//	bmin, bmax - (in) bounding box of the heightfield
//	cs - (in) grid cell size
//	ch - (in) grid cell height
//	walkableHeight - (in) minimum height where the agent can still walk
//	walkableClimb - (in) maximum height between grid cells the agent can climb
//	hf - (in) heightfield to be compacted
//	chf - (out) compact heightfield representing the open space.
// Returns false if operation ran out of memory.
bool rcBuildCompactHeightfield(const float* bmin, const float* bmax,
							   const float cs, const float ch,
							   const int walkableHeight, const int walkableClimb,
							   unsigned char flags,
							   rcHeightfield& hf,
							   rcCompactHeightfield& chf);

// Builds distance field and stores it into the combat heightfield.
// Params:
//	chf - (in/out) compact heightfield representing the open space.
// Returns false if operation ran out of memory.
bool rcBuildDistanceField(rcCompactHeightfield& chf);

// Divides the walkable heighfied into simple regions.
// Each region has only one contour and no overlaps.
// The regions are stored in the compact heightfield 'reg' field.
// The regions will be shrinked by the radius of the agent.
// The process sometimes creates small regions. The parameter
// 'minRegionSize' specifies the smallest allowed regions size.
// If the area of a regions is smaller than allowed, the regions is
// removed or merged to neighbour region. 
// Params:
//	chf - (in/out) compact heightfield representing the open space.
//	walkableRadius - (in) the radius of the agent.
//	minRegionSize - (in) the smallest allowed regions size.
//	maxMergeRegionSize - (in) the largest allowed regions size which can be merged.
// Returns false if operation ran out of memory.
bool rcBuildRegions(rcCompactHeightfield& chf,
					int walkableRadius, int borderSize, int minRegionSize, int mergeRegionSize);

// Builds simplified contours from the regions outlines.
// Params:
//	chf - (in) compact heightfield which has regions set.
//	maxError - (in) maximum allowed distance between simplified countour and cells.
//	maxEdgeLen - (in) maximum allowed contour edge length in cells.
//	cset - (out) Resulting contour set.
// Returns false if operation ran out of memory.
bool rcBuildContours(rcCompactHeightfield& chf,
					 float maxError, int maxEdgeLen,
					 rcContourSet& cset);

// Ensures that connected contour sets A and B share the same vertices at the shared edges.
// Params:
//  cseta - (in) contour set A.
//  csetb - (in) contour set B.
//  edge - (in) which edge to conform: 1) B is left of A  2) B is top of A
//  borderSize - (in) the border which was used when the contours were generated.
//  tileSize - (in) the tile size which was used when the contours were generated.
//	orig - (in) origin of the contour set A.
//	cs - (in) grid cell size
//	ch - (in) grid cell height
bool rcFixupAdjacentContours(rcContourSet* cseta, rcContourSet* csetb, int edge, int edgePos);

// Translates the cordinates of the contour set.
// Params:
//  cset - (in) contour set to translate.
//  dx - (in) delta X.
//  dy - (in) delta Y.
//  dz - (in) delta Z.
void rcTranslateContours(rcContourSet* cset, int dx, int dy, int dz);

// Builds connected convex polygon mesh from contour polygons.
// Params:
//	cset - (in) contour set.
//	mesh - (out) poly mesh.
//	nvp - (int) maximum number of vertices per polygon.
// Returns false if operation ran out of memory.
bool rcBuildPolyMesh(rcContourSet& cset,
					 const float* bmin, const float* bmax,
					 const float cs, const float ch, int nvp,
					 rcPolyMesh& mesh);

bool rcBuildNavMesh(const rcConfig& cfg,
					const float* verts, const int nverts,
					const int* tris, const unsigned char* tflags, const int ntris,
					rcHeightfield& solid,
					rcCompactHeightfield& chf,
					rcContourSet& cset,
					rcPolyMesh& polyMesh);


#endif // RECAST_H
