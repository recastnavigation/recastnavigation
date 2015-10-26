//
//  Heightfield.h
//  Recast
//
//  Created by Graham Pentheny on 7/31/15.
//
//

#ifndef Recast_Heightfield_h
#define Recast_Heightfield_h

/// @}
/// @name Heightfield Functions
/// @see rcHeightfield
/// @{

/// Calculates the bounding box of an array of vertices.
///  @ingroup recast
///  @param[in]		verts	An array of vertices. [(x, y, z) * @p nv]
///  @param[in]		nv		The number of vertices in the @p verts array.
///  @param[out]	bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[out]	bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax);

/// Calculates the grid size based on the bounding box and grid cell size.
///  @ingroup recast
///  @param[in]		bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]		bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]		cs		The xz-plane cell size. [Limit: > 0] [Units: wu]
///  @param[out]	w		The width along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[out]	h		The height along the z-axis. [Limit: >= 0] [Units: vx]
void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h);

/// Initializes a new heightfield.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in,out]	hf		The allocated heightfield to initialize.
///  @param[in]		width	The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[in]		height	The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
///  @param[in]		bmin	The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]		bmax	The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]		cs		The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
///  @param[in]		ch		The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height, const float* bmin, const float* bmax, float cs, float ch);

/// Sets the area id of all triangles with a slope below the specified value
/// to #RC_WALKABLE_AREA.
///  @ingroup recast
///  @param[in,out]	ctx					The build context to use during the operation.
///  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
///  									[Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv					The number of vertices.
///  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		nt					The number of triangles.
///  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv, const int* tris, int nt, unsigned char* areas);

/// Sets the area id of all triangles with a slope greater than or equal to the specified value to #RC_NULL_AREA.
///  @ingroup recast
///  @param[in,out]	ctx					The build context to use during the operation.
///  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
///  									[Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv					The number of vertices.
///  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		nt					The number of triangles.
///  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv, const int* tris, int nt, unsigned char* areas);

/// Adds a span to the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in,out]	hf				An initialized heightfield.
///  @param[in]		x				The width index where the span is to be added.
///  								[Limits: 0 <= value < rcHeightfield::width]
///  @param[in]		y				The height index where the span is to be added.
///  								[Limits: 0 <= value < rcHeightfield::height]
///  @param[in]		smin			The minimum height of the span. [Limit: < @p smax] [Units: vx]
///  @param[in]		smax			The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
///  @param[in]		area			The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
///  @param[in]		flagMergeThr	The merge theshold. [Limit: >= 0] [Units: vx]
void rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y, const unsigned short smin, const unsigned short smax, const unsigned char area, const int flagMergeThr);

/// Rasterizes a triangle into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		v0				Triangle vertex 0 [(x, y, z)]
///  @param[in]		v1				Triangle vertex 1 [(x, y, z)]
///  @param[in]		v2				Triangle vertex 2 [(x, y, z)]
///  @param[in]		area			The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
///  								[Limit: >= 0] [Units: vx]
void rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2, const unsigned char area, rcHeightfield& solid, const int flagMergeThr = 1);

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		verts			The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv				The number of vertices.
///  @param[in]		tris			The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt				The number of triangles.
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
///  								[Limit: >= 0] [Units: vx]
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv, const int* tris, const unsigned char* areas, const int nt, rcHeightfield& solid, const int flagMergeThr = 1);

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx			The build context to use during the operation.
///  @param[in]		verts		The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv			The number of vertices.
///  @param[in]		tris		The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		areas		The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt			The number of triangles.
///  @param[in,out]	solid		An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
///  							[Limit: >= 0] [Units: vx]
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv, const unsigned short* tris, const unsigned char* areas, const int nt, rcHeightfield& solid, const int flagMergeThr = 1);

/// Rasterizes triangles into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		verts			The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
///  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt				The number of triangles.
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
///  								[Limit: >= 0] [Units: vx]
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt, rcHeightfield& solid, const int flagMergeThr = 1);

/// Marks non-walkable spans as walkable if their maximum is within @p walkableClimp of a walkable neihbor.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable.
///  								[Limit: >=0] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid);

/// Marks spans that are ledges as not-walkable.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to
///  								be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable.
///  								[Limit: >=0] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid);

/// Marks walkable spans as not walkable if the clearence above the span is less than the specified height.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to
///  								be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid);

/// Returns the number of spans contained in the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		hf		An initialized heightfield.
///  @returns The number of spans in the heightfield.
int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf);


#endif
