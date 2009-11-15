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

#ifndef RECAST_DEBUGDRAW_H
#define RECAST_DEBUGDRAW_H

enum rcDebugDrawPrimitives
{
	RC_DRAW_POINTS,
	RC_DRAW_LINES,
	RC_DRAW_TRIS,
	RC_DRAW_QUADS,	
};

// Abstrace debug draw interface.
struct rcDebugDraw
{
	// Begin drawing primitives.
	// Params:
	//  prim - (in) primitive type to draw, one of rcDebugDrawPrimitives.
	//  nverts - (in) number of vertices to be submitted.
	//  size - (in) size of a primitive, applies to point size and line width only.
	virtual void begin(rcDebugDrawPrimitives prim, int nverts, float size = 1.0f) = 0;

	// Submit a vertex
	// Params:
	//  pos - (in) position of the verts.
	//  color - (in) color of the verts.
	virtual void vertex(const float* pos, unsigned int color) = 0;

	// Submit a vertex
	// Params:
	//  x,y,z - (in) position of the verts.
	//  color - (in) color of the verts.
	virtual void vertex(const float x, const float y, const float z, unsigned int color) = 0;

	// End drawing primitives.
	virtual void end() = 0;
};

inline unsigned int RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	return (r) | (g << 8) | (b << 16) | (a << 24);
}

inline unsigned int RGBAf(float fr, float fg, float fb, float fa)
{
	unsigned char r = (unsigned char)(fr*255.0f);
	unsigned char g = (unsigned char)(fg*255.0f);
	unsigned char b = (unsigned char)(fb*255.0f);
	unsigned char a = (unsigned char)(fa*255.0f);
	return RGBA(r,g,b,a);
}

inline int bit(int a, int b)
{
	return (a & (1 << b)) >> b;
}

inline unsigned int intToCol(int i, int a)
{
	int	r = bit(i, 0) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 1) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 2) + bit(i, 5) * 2 + 1;
	return RGBA(r*63,g*63,b*63,a);
}

inline void intToCol(int i, float* col)
{
	int	r = bit(i, 0) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 1) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 2) + bit(i, 5) * 2 + 1;
	col[0] = 1 - r*63.0f/255.0f;
	col[1] = 1 - g*63.0f/255.0f;
	col[2] = 1 - b*63.0f/255.0f;
}

void rcDebugDrawHeightfieldSolid(rcDebugDraw* dd, const struct rcHeightfield& hf);
void rcDebugDrawHeightfieldWalkable(rcDebugDraw* dd, const struct rcHeightfield& hf);

void rcDebugDrawMesh(rcDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, const unsigned char* flags);
void rcDebugDrawMeshSlope(rcDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, const float walkableSlopeAngle);

void rcDebugDrawCompactHeightfieldSolid(rcDebugDraw* dd, const struct rcCompactHeightfield& chf);
void rcDebugDrawCompactHeightfieldRegions(rcDebugDraw* dd, const struct rcCompactHeightfield& chf);
void rcDebugDrawCompactHeightfieldDistance(rcDebugDraw* dd, const struct rcCompactHeightfield& chf);

void rcDebugDrawRegionConnections(rcDebugDraw* dd, const struct rcContourSet& cset, const float alpha = 1.0f);
void rcDebugDrawRawContours(rcDebugDraw* dd, const struct rcContourSet& cset, const float alpha = 1.0f);
void rcDebugDrawContours(rcDebugDraw* dd, const struct rcContourSet& cset, const float alpha = 1.0f);
void rcDebugDrawPolyMesh(rcDebugDraw* dd, const struct rcPolyMesh& mesh);
void rcDebugDrawPolyMeshDetail(rcDebugDraw* dd, const struct rcPolyMeshDetail& dmesh);

void rcDebugDrawCylinderWire(rcDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, const float* col);
void rcDebugDrawBoxWire(rcDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, const float* col);
void rcDebugDrawBox(rcDebugDraw* dd, float minx, float miny, float minz, float maxx, float maxy, float maxz,
					const float* col1, const float* col2);
void rcDrawArc(rcDebugDraw* dd, const float* p0, const float* p1, const float* col, float lineWidth);

#endif // RECAST_DEBUGDRAW_H
