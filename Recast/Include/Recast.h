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

/** 
 * @defgroup recast Recast
 * Elements related to path planning.
 * @note This list is not yet complete.  (The documentation effort is still underway.)
 */

/// The value of PI used by Recast.
static const float RC_PI = 3.14159265f;

/// Recast log categories.
/// @ingroup recast
/// @see rcContext
enum rcLogCategory
{
	RC_LOG_PROGRESS = 1,	///< A progress log entry.
	RC_LOG_WARNING,	 	    ///< A warning log entry.
	RC_LOG_ERROR,	   	    ///< An error log entry.
};

/// Recast performance timer categories.
/// @ingroup recast
/// @see rcContext
enum rcTimerLabel
{
	/// The user defined total time of the build.
	RC_TIMER_TOTAL,
	/// A user defined build time.
	RC_TIMER_TEMP,
	/// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
	RC_TIMER_RASTERIZE_TRIANGLES,
	/// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
	RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
	/// The total time to build the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS,
	/// The time to trace the boundaries of the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_TRACE,
	/// The time to simplify the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
	/// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
	RC_TIMER_FILTER_BORDER,
	/// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
	RC_TIMER_FILTER_WALKABLE,
	/// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
	RC_TIMER_MEDIAN_AREA,
	/// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
	RC_TIMER_FILTER_LOW_OBSTACLES,
	/// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
	RC_TIMER_BUILD_POLYMESH,
	/// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
	RC_TIMER_MERGE_POLYMESH,
	/// The time to erode the walkable area. (See: #rcErodeWalkableArea)
	RC_TIMER_ERODE_AREA,
	/// The time to mark a box area. (See: #rcMarkBoxArea)
	RC_TIMER_MARK_BOX_AREA,
	/// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
	RC_TIMER_MARK_CYLINDER_AREA,
	/// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
	RC_TIMER_MARK_CONVEXPOLY_AREA,
	/// The total time to build the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD,
	/// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_DIST,
	/// The time to blur the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
	/// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS,
	/// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_WATERSHED,
	/// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_EXPAND,
	/// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_FLOOD,
	/// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS_FILTER,
	/// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
	RC_TIMER_BUILD_LAYERS, 
	/// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
	RC_TIMER_BUILD_POLYMESHDETAIL,
	/// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
	RC_TIMER_MERGE_POLYMESHDETAIL,
	/// The maximum number of timers.  (Used for iterating timers.)
	RC_MAX_TIMERS
};

/// Provides an interface for optional logging and performance tracking of the Recast 
/// build process.
class rcContext
{
public:

	/// Contructor.
	///  @param[in] state  TRUE if the logging and performance timers should be enabled.  [Default: true]
	inline rcContext(bool state = true) : m_logEnabled(state), m_timerEnabled(state) {}
	virtual ~rcContext() {}

	/// Enables or disables logging.
	///  @param[in] state TRUE if logging should be enabled.
	inline void enableLog(bool state) { m_logEnabled = state; }

	/// Clears all log entries.
	inline void resetLog() { if (m_logEnabled) doResetLog(); }

	/// Logs a message.
	///  @param[in] category The category of the message.
	///  @param[in] format The message.
	void log(const rcLogCategory category, const char* format, ...);

	/// Enables or disables the performance timers.
	///  @param[in] state  TRUE if timers should be enabled.
	inline void enableTimer(bool state) { m_timerEnabled = state; }

	/// Clears all peformance timers. (Resets all to unused.)
	inline void resetTimers() { if (m_timerEnabled) doResetTimers(); }

	/// Starts the specified performance timer.
	///  @param label  The category of timer.
	inline void startTimer(const rcTimerLabel label) { if (m_timerEnabled) doStartTimer(label); }

	/// Stops the specified performance timer.
	///  @param label  The category of the timer.
	inline void stopTimer(const rcTimerLabel label) { if (m_timerEnabled) doStopTimer(label); }

	/// Returns the total accumulated time of the specified performance timer.
	///  @param label  The category of the timer.
	///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	inline int getAccumulatedTime(const rcTimerLabel label) const { return m_timerEnabled ? doGetAccumulatedTime(label) : -1; }

protected:

	/// @name Custom implementation functions.
	/// Logging and timer functionality must be provided by a concrete
	/// implementation of these functions. This class does not implement these functions.
	///@{

	/// Clears all log entries.
	virtual void doResetLog() {}

	/// Logs a message.
	///  @param[in] category The category of the message.
	///  @param[in] msg The formatted message.
	///  @param[in] len The length of the formatted message.
	virtual void doLog(const rcLogCategory /*category*/, const char* /*msg*/, const int /*len*/) {}

	/// Clears all timers. (Resets all to unused.)
	virtual void doResetTimers() {}

	/// Starts the specified performance timer.
	///  @param[in] label  The category of timer.
	virtual void doStartTimer(const rcTimerLabel /*label*/) {}

	/// Stops the specified performance timer.
	///  @param[in] label  The category of the timer.
	virtual void doStopTimer(const rcTimerLabel /*label*/) {}

	/// Returns the total accumulated time of the specified performance timer.
	///  @param[in] label  The category of the timer.
	///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	virtual int doGetAccumulatedTime(const rcTimerLabel /*label*/) const { return -1; }

	///@}
	
	/// True if logging is enabled.
	bool m_logEnabled;

	/// True if the performance timers are enabled.
	bool m_timerEnabled;
};

/// Specifies a configuration to use when performing Recast builds.
struct rcConfig
{
	/// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	int width;

	/// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	int height;	
	
	/// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	int tileSize;	
	
	/// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	int borderSize;

	/// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] 
	float cs;

	/// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	float ch;

	/// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	float bmin[3]; 

	/// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	float bmax[3];

	/// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] 
	float walkableSlopeAngle;

	/// Minimum floor to 'ceiling' height that will still allow the floor area to 
	/// be considered walkable. [Limit: >= 3] [Units: vx] 
	int walkableHeight;				
	
	/// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] 
	int walkableClimb;				
	
	/// The distance to erode/shrink the walkable area of the heightfield away from 
	/// obstructions.  [Limit: >=0] [Units: vx] 
	int walkableRadius;				
	
	/// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] 
	int maxEdgeLen;					
	
	/// The maximum distance a simplfied contour's border edges should deviate 
	/// the original raw contour. [Limit: >=0] [Units: wu]
	float maxSimplificationError;	
	
	/// The minimum number of cells allowed to form isolated island regions. [Limit: >=0] [Units: vx] 
	int minRegionArea;				
	
	/// Any regions with a cell count smaller than this value will, if possible, 
	/// be merged with larger regions. [Limit: >=0] [Units: vx] 
	int mergeRegionArea;			
	
	/// The maximum number of vertices allowed for polygons generated during the 
	/// contour to polygon conversion process. [Limit: >= 3] 
	int maxVertsPerPoly;
	
	/// Sets the sampling distance to use when generating the detail mesh.
	/// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu] 
	float detailSampleDist;
	
	/// The maximum distance the detail mesh surface should deviate from heightfield
	/// data. (For height detail only.) [Limit: >=0] [Units: wu] 
	float detailSampleMaxError;
};

/// Defines number of bits in rcSpan::smin and rcSpan::smax.
static const int RC_SPAN_HEIGHT_BITS = 13;
/// Defines the maximum value for rcSpan::smin and rcSpan::smax.
static const int RC_SPAN_MAX_HEIGHT = (1<<RC_SPAN_HEIGHT_BITS)-1;

/// Represents a span in a heightfield.
/// @see rcHeightfield
struct rcSpan
{
	unsigned int smin : 13;			///< The mimum height of the span.
	unsigned int smax : 13;			///< The maximum height of the span.
	unsigned int area : 6;			///< The area id assigned to the span.
	rcSpan* next;					///< The next span higher up in column.
};

/// The number of spans allocated per span spool.
/// @see rcSpanPool
static const int RC_SPANS_PER_POOL = 2048;

/// A memory pool used for quick allocation of spans within a heightfield.
/// @see rcHeightfield
struct rcSpanPool
{
	rcSpanPool* next;					///< The next span pool.
	rcSpan items[RC_SPANS_PER_POOL];	///< Array of spans in the pool.
};

/// A dynamic heightfield representing obstructed space.
struct rcHeightfield
{
	int width;	  	    ///< The width of the heightfield. (Along the x-axis in cell units.)
	int height;			///< The height of the heightfield. (Along the z-axis in cell units.)
	float bmin[3];  	///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];		///< The maximum bounds in world space. [(x, y, z)]
	float cs;	   	    ///< The size of each cell. (On the xz-plane.)
	float ch;			///< The height of each cell. (The minimum increment along the y-axis.)
	rcSpan** spans;		///< Heightfield of spans (width*height).
	rcSpanPool* pools;	///< Linked list of span pools.
	rcSpan* freelist;	///< The next free span.
};

/// Allocates a heightfield object using the Recast allocator.
///  @return A heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
rcHeightfield* rcAllocHeightfield();

/// Frees the specified heightfield object using the Recast allocator.
///  @param[in] hf  A heightfield allocated using #rcAllocHeightfield
///  @ingroup recast
void rcFreeHeightField(rcHeightfield* hf);


/// Provides information on the content of a cell column in a compact heightfield. 
struct rcCompactCell
{
	unsigned int index : 24;	///< Index to the first span in the column.
	unsigned int count : 8;		///< Number of spans in the column.
};

/// Represents a span of unobstructed space within a compact heightfield.
struct rcCompactSpan
{
	unsigned short y;			///< The lower extent of the span. (Measured from the heightfield's base.)
	unsigned short reg;	 	    ///< The id of the region the span belongs to. (Or zero if not in a region.)
	unsigned int con : 24;		///< Packed neighbor connection data.
	unsigned int h : 8;			///< The height of the span.  (Measured from #y.)
};

/// A compact, static heightfield representing unobstructed space.
struct rcCompactHeightfield
{
	int width;			  	    ///< The width of the heightfield. (Along the x-axis in cell units.)
	int height;					///< The height of the heightfield. (Along the z-axis in cell units.)
	int spanCount;				///< The number of spans in the heightfield.
	int walkableHeight;	 	    ///< The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
	int walkableClimb;			///< The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
	int borderSize;				///< The AABB border size used during the build of the field. (See: rcConfig::borderSize)
	unsigned short maxDistance;	///< The maximum distance value of any span within the field. 
	unsigned short maxRegions;	///< The maximum region id of any span within the field. 
	float bmin[3];		  	    ///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];				///< The maximum bounds in world space. [(x, y, z)]
	float cs;			   	    ///< The size of each cell. (On the xz-plane.)
	float ch;					///< The height of each cell. (The minimum increment along the y-axis.)
	rcCompactCell* cells;		///< Array of cells. [Size: #width*#height]
	rcCompactSpan* spans;		///< Array of spans. [Size: #spanCount]
	unsigned short* dist;		///< Array containing border distance data. [Size: #spanCount]
	unsigned char* areas;		///< Array containing area id data. [Size: #spanCount]
};

/// Allocates a compact heightfield object using the Recast allocator.
///  @return A compact heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
rcCompactHeightfield* rcAllocCompactHeightfield();

/// Frees the specified compact heightfield object using the Recast allocator.
///  @param[in] chf  A compact heightfield allocated using #rcAllocCompactHeightfield
///  @ingroup recast
void rcFreeCompactHeightfield(rcCompactHeightfield* chf);


/// Represents a heightfield layer within a layer set.
/// @ingroup recast
/// @see rcHeightfieldLayerSet
struct rcHeightfieldLayer
{
	float bmin[3];		  	    ///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];				///< The maximum bounds in world space. [(x, y, z)]
	float cs;			   	    ///< The size of each cell. (On the xz-plane.)
	float ch;					///< The height of each cell. (The minimum increment along the y-axis.)
	int width;			  	    ///< The width of the heightfield. (Along the x-axis in cell units.)
	int height;					///< The height of the heightfield. (Along the z-axis in cell units.)
	int minx;			   	    ///< The minimum x-bounds of usable data.
	int maxx;			   	    ///< The maximum x-bounds of usable data.
	int miny;			   	    ///< The minimum y-bounds of usable data. (Along the z-axis.)
	int maxy;					///< The maximum y-bounds of usable data. (Along the z-axis.)
	int hmin;			   	    ///< The minimum height bounds of usable data. (Along the y-axis.)
	int hmax;					///< The maximum height bounds of usable data. (Along the y-axis.)
	unsigned char* heights;		///< The heightfield. [Size: (width - borderSize*2) * (h - borderSize*2)]
	unsigned char* areas;		///< Area ids. [Size: Same as #heights]
	unsigned char* cons;		///< Packed neighbor connection information. [Size: Same as #heights]
};

/// Represents a set of heightfield layers.
/// @ingroup recast
/// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet 
struct rcHeightfieldLayerSet
{
	rcHeightfieldLayer* layers;			///< The layers in the set. [Size: #nlayers]
	int nlayers;						///< The number of layers in the set.
};

/// Allocates a heightfield layer set using the Recast allocator.
///  @return A heightfield layer set that is ready for initialization, or null on failure.
///  @ingroup recast
rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet();

/// Frees the specified heightfield layer set using the Recast allocator.
///  @param[in] lset  A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
///  @ingroup recast
void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset);

/// Represents a simple, non-overlapping contour in field space.
struct rcContour
{
	int* verts;			///< Simplified contour vertex and connection data. [Size: 4 * #nverts]
	int nverts;			///< The number of vertices in the simplified contour. 
	int* rverts;		///< Raw contour vertex and connection data. [Size: 4 * #nrverts]
	int nrverts;		///< The number of vertices in the raw contour. 
	unsigned short reg;	///< The region id of the contour.
	unsigned char area;	///< The area id of the contour.
};

/// Represents a group of related contours.
struct rcContourSet
{
	rcContour* conts;	///< An array of the contours in the set. [Size: #nconts]
	int nconts;			///< The number of contours in the set.
	float bmin[3];  	///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];		///< The maximum bounds in world space. [(x, y, z)]
	float cs;	   	    ///< The size of each cell. (On the xz-plane.)
	float ch;			///< The height of each cell. (The minimum increment along the y-axis.)
	int width;	  	    ///< The width of the set. (Along the x-axis in cell units.) 
	int height;	 	    ///< The height of the set. (Along the z-axis in cell units.) 
	int borderSize;		///< The AABB border size used to generate the source data from which the contours were derived.
};

/// Allocates a contour set object using the Recast allocator.
///  @return A contour set that is ready for initialization, or null on failure.
///  @ingroup recast
rcContourSet* rcAllocContourSet();

/// Frees the specified contour set using the Recast allocator.
///  @param[in] cset  A contour set allocated using #rcAllocContourSet
///  @ingroup recast
void rcFreeContourSet(rcContourSet* cset);


/// Represents a polygon mesh suitable for use in building a navigation mesh. 
struct rcPolyMesh
{	
	unsigned short* verts;	///< The mesh vertices. [Form: (x, y, z) * #nverts]
	unsigned short* polys;	///< Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	unsigned short* regs;	///< The region id assigned to each polygon. [Length: #maxpolys]
	unsigned short* flags;	///< The user defined flags for each polygon. [Length: #maxpolys]
	unsigned char* areas;	///< The area id assigned to each polygon. [Length: #maxpolys]
	int nverts;				///< The number of vertices.
	int npolys;				///< The number of polygons.
	int maxpolys;			///< The number of allocated polygons.
	int nvp;				///< The maximum number of vertices per polygon.
	float bmin[3];	  	    ///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];	  	    ///< The maximum bounds in world space. [(x, y, z)]
	float cs;		   	    ///< The size of each cell. (On the xz-plane.)
	float ch;				///< The height of each cell. (The minimum increment along the y-axis.)
	int borderSize;			///< The AABB border size used to generate the source data from which the mesh was derived.
};

/// Allocates a polygon mesh object using the Recast allocator.
///  @return A polygon mesh that is ready for initialization, or null on failure.
///  @ingroup recast
rcPolyMesh* rcAllocPolyMesh();

/// Frees the specified polygon mesh using the Recast allocator.
///  @param[in] pmesh  A polygon mesh allocated using #rcAllocPolyMesh
///  @ingroup recast
void rcFreePolyMesh(rcPolyMesh* pmesh);


/// Detail mesh generated from a rcPolyMesh.
/// Each submesh represents a polygon in the polymesh and they are stored in
/// exactly same order. Each submesh is described as 4 values:
/// base vertex, vertex count, base triangle, triangle count. That is,
///   const unsigned char* t = &dmesh.tris[(tbase+i)*4]; and
///   const float* v = &dmesh.verts[(vbase+t[j])*3];
/// If the input polygon has 'n' vertices, those vertices are first in the
/// submesh vertex list. This allows to compres the mesh by not storing the
/// first vertices and using the polymesh vertices instead.
/// Max number of vertices per submesh is 127 and
/// max number of triangles per submesh is 255.
struct rcPolyMeshDetail
{
	unsigned int* meshes;	///< Pointer to all mesh data.
	float* verts;			///< Pointer to all vertex data.
	unsigned char* tris;	///< Pointer to all triangle data.
	int nmeshes;			///< Number of meshes.
	int nverts;				///< Number of total vertices.
	int ntris;				///< Number of triangles.
};

rcPolyMeshDetail* rcAllocPolyMeshDetail();
void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh);


/// If heightfield region ID has the following bit set, the region is on border area
/// and excluded from many calculations.
static const unsigned short RC_BORDER_REG = 0x8000;

/// If contour region ID has the following bit set, the vertex will be later
/// removed in order to match the segments and vertices at tile boundaries.
static const int RC_BORDER_VERTEX = 0x10000;

static const int RC_AREA_BORDER = 0x20000;

enum rcBuildContoursFlags
{
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,	///< Tessellate wall edges
	RC_CONTOUR_TESS_AREA_EDGES = 0x02,	///< Tessellate edges between areas.
};

/// Mask used with contours to extract region id.
static const int RC_CONTOUR_REG_MASK = 0xffff;

/// Null index which is used with meshes to mark unset or invalid indices.
static const unsigned short RC_MESH_NULL_IDX = 0xffff;

/// Area ID that is considered empty.
static const unsigned char RC_NULL_AREA = 0;

/// Area ID that is considered generally walkable.
static const unsigned char RC_WALKABLE_AREA = 63;

/// Value returned by rcGetCon() if the direction is not connected.
static const int RC_NOT_CONNECTED = 0x3f;

/// Compact span neighbour helpers.
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

/// @name Common helper functions
///@{
template<class T> inline void rcSwap(T& a, T& b) { T t = a; a = b; b = t; }
template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }
template<class T> inline T rcMax(T a, T b) { return a > b ? a : b; }
template<class T> inline T rcAbs(T a) { return a < 0 ? -a : a; }
template<class T> inline T rcSqr(T a) { return a*a; }
template<class T> inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }
float rcSqrt(float x);
inline int rcAlign4(int x) { return (x+3) & ~3; }
///@}

/// @name Common vector helper functions.
///@{
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
///@}

/// Calculated bounding box of array of vertices.
///  @param verts [in] array of vertices
///  @param nv [in] vertex count
///  @param bmin,bmax [out] bounding box
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax);

/// Calculates grid size based on bounding box and grid cell size.
///  @param bmin,bmax [in] bounding box
///  @param cs [in] grid cell size
///  @param w [out] grid width
///  @param h [out] grid height
void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h);

/// Creates and initializes new heightfield.
///  @param hf [in,out] heightfield to initialize.
///  @param width [in] width of the heightfield.
///  @param height [in] height of the heightfield.
///  @param bmin,bmax [in] bounding box of the heightfield
///  @param cs [in] grid cell size
///  @param ch [in] grid cell height
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height,
						 const float* bmin, const float* bmax,
						 float cs, float ch);

/// Sets the RC_WALKABLE_AREA for every triangle whose slope is below
/// the maximum walkable slope angle.
///  @param walkableSlopeAngle [in] maximum slope angle in degrees.
///  @param verts [in] array of vertices
///  @param nv [in] vertex count
///  @param tris [in] array of triangle vertex indices
///  @param nt [in] triangle count
///  @param areas [out] array of triangle area types
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
							 const int* tris, int nt, unsigned char* areas); 

/// Sets the RC_NULL_AREA for every triangle whose slope is steeper than
/// the maximum walkable slope angle.
///  @param walkableSlopeAngle [in] maximum slope angle in degrees.
///  @param verts [in] array of vertices
///  @param nv [in] vertex count
///  @param tris [in] array of triangle vertex indices
///  @param nt [in] triangle count
///  @param areas [out] array of triangle are types
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
								const int* tris, int nt, unsigned char* areas); 

/// Adds span to heightfield.
/// The span addition can set to favor flags. If the span is merged to
/// another span and the new smax is within 'flagMergeThr' units away
/// from the existing span the span flags are merged and stored.
///  @param x,y [in] location on the heightfield where the span is added
///  @param smin,smax [in] spans min/max height
///  @param area
///  @param flagMergeThr [in] merge threshold.
void rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y,
			   const unsigned short smin, const unsigned short smax,
			   const unsigned char area, const int flagMergeThr);

/// Rasterizes a triangle into heightfield spans.
///  @param v0,v1,v2 [in] the vertices of the triangle.
///  @param area [in] area type of the triangle.
///  @param solid [in] heightfield where the triangle is rasterized
///  @param flagMergeThr [in] distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& solid,
						 const int flagMergeThr = 1);

/// Rasterizes indexed triangle mesh into heightfield spans.
///  @param verts [in] array of vertices
///  @param nv [in] vertex count
///  @param tris [in] array of triangle vertex indices
///  @param areas [in] array of triangle area types.
///  @param nt [in] triangle count
///  @param solid [in] heightfield where the triangles are rasterized
///  @param flagMergeThr [in] distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

/// Rasterizes indexed triangle mesh into heightfield spans.
///  @param verts [in] array of vertices
///  @param nv [in] vertex count
///  @param tris [in] array of triangle vertex indices
///  @param areas [in] array of triangle area types.
///  @param nt [in] triangle count
///  @param solid [in] heightfield where the triangles are rasterized
///  @param flagMergeThr [in] distance in voxel where walkable flag is favored over non-walkable.
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
						  const unsigned short* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

/// Rasterizes the triangles into heightfield spans.
///  @param verts [in] array of vertices
///  @param areas [in] array of triangle area types.
///  @param nt [in] triangle count
///  @param solid [in] heightfield where the triangles are rasterized
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr = 1);

/// Marks non-walkable low obstacles as walkable if they are closer than walkableClimb
/// from a walkable surface. Applying this filter allows to step over low hanging
/// low obstacles.
///  @param walkableClimb [in] maximum height between grid cells the agent can climb
///  @param solid [in,out] heightfield describing the solid space
/// @warning TODO: Misses ledge flag, must be called before rcFilterLedgeSpans!
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid);

/// Removes WALKABLE flag from all spans that are at ledges. This filtering
/// removes possible overestimation of the conservative voxelization so that
/// the resulting mesh will not have regions hanging in air over ledges.
///  @param walkableHeight [in] minimum height where the agent can still walk
///  @param walkableClimb [in] maximum height between grid cells the agent can climb
///  @param solid [in,out] heightfield describing the solid space
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight,
						const int walkableClimb, rcHeightfield& solid);

/// Removes WALKABLE flag from all spans which have smaller than
/// 'walkableHeight' clearance above them.
///  @param walkableHeight [in] minimum height where the agent can still walk
///  @param solid [in,out] heightfield describing the solid space
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid);

/// Returns number of spans contained in a heightfield.
///  @param hf [in] heightfield to be compacted
///  @returns number of spans.
int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf);

/// Builds compact representation of the heightfield.
///  @param walkableHeight [in] minimum height where the agent can still walk
///  @param walkableClimb [in] maximum height between grid cells the agent can climb
///  @param hf [in] heightfield to be compacted
///  @param chf [out] compact heightfield representing the open space.
///  @returns false if operation ran out of memory.
bool rcBuildCompactHeightfield(rcContext* ctx, const int walkableHeight, const int walkableClimb,
							   rcHeightfield& hf, rcCompactHeightfield& chf);

/// Erodes walkable area.
///  @param radius [in] radius of erosion (max 255).
///  @param chf [in,out] compact heightfield to erode.
///  @returns false if operation ran out of memory.
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& chf);

/// Applies median filter to walkable area types, removing noise.
///  @param chf [in,out] compact heightfield to erode.
///  @returns false if operation ran out of memory.
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf);

/// Marks the area of the convex polygon into the area type of the compact heightfield.
///  @param bmin,bmax [in] bounds of the axis aligned box.
///  @param areaId [in] area ID to mark.
///  @param chf [in,out] compact heightfield to mark.
void rcMarkBoxArea(rcContext* ctx, const float* bmin, const float* bmax, unsigned char areaId,
				   rcCompactHeightfield& chf);

/// Marks the area of the convex polygon into the area type of the compact heightfield.
///  @param verts [in] vertices of the convex polygon.
///  @param nverts [in] number of vertices in the polygon.
///  @param hmin,hmax [in] min and max height of the polygon.
///  @param areaId [in] area ID to mark.
///  @param chf [in,out] compact heightfield to mark.
void rcMarkConvexPolyArea(rcContext* ctx, const float* verts, const int nverts,
						  const float hmin, const float hmax, unsigned char areaId,
						  rcCompactHeightfield& chf);

/// Marks the area of the cylinder into the area type of the compact heightfield.
///  @param pos [in] center bottom location of hte cylinder.
///  @param r [in] radius of the cylinder.
///  @param h [in] height of the cylinder.
///  @param areaId [in] area ID to mark.
///  @param chf [in,out] compact heightfield to mark.
void rcMarkCylinderArea(rcContext* ctx, const float* pos,
						const float r, const float h, unsigned char areaId,
						rcCompactHeightfield& chf);

/// Builds distance field and stores it into the combat heightfield.
///  @param chf [in,out] compact heightfield representing the open space.
///  @returns false if operation ran out of memory.
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf);

/// Divides the walkable heighfied into simple regions using watershed partitioning.
/// Each region has only one contour and no overlaps.
/// The regions are stored in the compact heightfield 'reg' field.
/// The process sometimes creates small regions. If the area of a regions is
/// smaller than 'mergeRegionArea' then the region will be merged with a neighbour
/// region if possible. If multiple regions form an area which is smaller than
/// 'minRegionArea' all the regions belonging to that area will be removed.
/// Here area means the count of spans in an area.
///  @param chf [in,out] compact heightfield representing the open space.
///  @param borderSize [in] Non-navigable Border around the heightfield.
///  @param minRegionArea [in] the smallest allowed region area.
///  @param maxMergeRegionArea [in] the largest allowed region area which can be merged.
///  @returns false if operation ran out of memory.
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionArea, const int mergeRegionArea);

/// Divides the walkable heighfied into simple regions using simple monotone partitioning.
/// Each region has only one contour and no overlaps.
/// The regions are stored in the compact heightfield 'reg' field.
/// The process sometimes creates small regions. If the area of a regions is
/// smaller than 'mergeRegionArea' then the region will be merged with a neighbour
/// region if possible. If multiple regions form an area which is smaller than
/// 'minRegionArea' all the regions belonging to that area will be removed.
/// Here area means the count of spans in an area.
///  @param chf [in,out] compact heightfield representing the open space.
///  @param borderSize [in] Non-navigable Border around the heightfield.
///  @param minRegionArea [in] the smallest allowed regions size.
///  @param maxMergeRegionArea [in] the largest allowed regions size which can be merged.
///  @returns false if operation ran out of memory.
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea);

/// Builds 2D layer representation of a heighfield.
///  @param chf [in] compact heightfield representing the open space.
///  @param borderSize [in] Non-navigable Border around the heightfield.
///  @param walkableHeight [in] minimum height where the agent can still walk.
///  @param lset [out] set of 2D heighfield layers.
///  @returns false if operation ran out of memory.
bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf, 
							  const int borderSize, const int walkableHeight,
							  rcHeightfieldLayerSet& lset);

/// Builds simplified contours from the regions outlines.
///  @param chf [in] compact heightfield which has regions set.
///  @param maxError [in] maximum allowed distance between simplified contour and cells.
///  @param maxEdgeLen [in] maximum allowed contour edge length in cells.
///  @param cset [out] Resulting contour set.
///  @param flags [in] build flags, see rcBuildContoursFlags.
///  @returns false if operation ran out of memory.
bool rcBuildContours(rcContext* ctx, rcCompactHeightfield& chf,
					 const float maxError, const int maxEdgeLen,
					 rcContourSet& cset, const int flags = RC_CONTOUR_TESS_WALL_EDGES);

/// Builds connected convex polygon mesh from contour polygons.
///  @param cset [in] contour set.
///  @param nvp [in] maximum number of vertices per polygon.
///  @param mesh [out] poly mesh.
///  @returns false if operation ran out of memory.
bool rcBuildPolyMesh(rcContext* ctx, rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

bool rcMergePolyMeshes(rcContext* ctx, rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh);

/// Builds detail triangle mesh for each polygon in the poly mesh.
///  @param mesh [in] poly mesh to detail.
///  @param chf [in] compact height field, used to query height for new vertices.
///  @param sampleDist [in] spacing between height samples used to generate more detail into mesh.
///  @param sampleMaxError [in] maximum allowed distance between simplified detail mesh and height sample.
///  @param dmesh [out] detail mesh.
///  @returns false if operation ran out of memory.
bool rcBuildPolyMeshDetail(rcContext* ctx, const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
						   const float sampleDist, const float sampleMaxError,
						   rcPolyMeshDetail& dmesh);

bool rcMergePolyMeshDetails(rcContext* ctx, rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh);



#endif // RECAST_H

///////////////////////////////////////////////////////////////////////////

// Due to the large amount of detail documentation for this file, 
// the content normally located at the end of the header file has been separated
// out to a file in the /Docs/Extern directory.
