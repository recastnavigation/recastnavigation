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

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

enum duDebugDrawPrimitives
{
	DU_DRAW_POINTS,
	DU_DRAW_LINES,
	DU_DRAW_TRIS,
	DU_DRAW_QUADS,	
};

// Abstrace debug draw interface.
struct duDebugDraw
{
	virtual ~duDebugDraw() = 0;
	
	virtual void depthMask(bool state) = 0;

	// Begin drawing primitives.
	// Params:
	//  prim - (in) primitive type to draw, one of rcDebugDrawPrimitives.
	//  nverts - (in) number of vertices to be submitted.
	//  size - (in) size of a primitive, applies to point size and line width only.
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f) = 0;

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

inline unsigned int duRGBA(int r, int g, int b, int a)
{
	return ((unsigned int)r) | ((unsigned int)g << 8) | ((unsigned int)b << 16) | ((unsigned int)a << 24);
}

inline unsigned int duRGBAf(float fr, float fg, float fb, float fa)
{
	unsigned char r = (unsigned char)(fr*255.0f);
	unsigned char g = (unsigned char)(fg*255.0f);
	unsigned char b = (unsigned char)(fb*255.0f);
	unsigned char a = (unsigned char)(fa*255.0f);
	return duRGBA(r,g,b,a);
}

unsigned int duIntToCol(int i, int a);
void duIntToCol(int i, float* col);

inline unsigned int duDarkenColor(unsigned int col)
{
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000);
}

void duCalcBoxColors(unsigned int* colors, unsigned int colTop, unsigned int colSide);

void duDebugDrawCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, unsigned int col, const float lineWidth);

void duDebugDrawBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, unsigned int col, const float lineWidth);

void duDebugDrawArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					const float x1, const float y1, const float z1, const float h,
					const float as0, const float as1, unsigned int col, const float lineWidth);

void duDebugDrawCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					   const float r, unsigned int col, const float lineWidth);

void duDebugDrawCross(struct duDebugDraw* dd, const float x, const float y, const float z,
					  const float size, unsigned int col, const float lineWidth);

void duDebugDrawBox(struct duDebugDraw* dd, float minx, float miny, float minz,
					float maxx, float maxy, float maxz, const unsigned int* fcol);

void duDebugDrawGridXZ(struct duDebugDraw* dd, const float ox, const float oy, const float oz,
					   const int w, const int h, const float size,
					   const unsigned int col, const float lineWidth);


// Versions without begin/end, can be used to draw multiple primitives.
void duAppendCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						  float maxx, float maxy, float maxz, unsigned int col);

void duAppendBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
					 float maxx, float maxy, float maxz, unsigned int col);

void duAppendBoxPoints(struct duDebugDraw* dd, float minx, float miny, float minz,
					   float maxx, float maxy, float maxz, unsigned int col);

void duAppendArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				 const float x1, const float y1, const float z1, const float h,
				 const float as0, const float as1, unsigned int col);

void duAppendCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					const float r, unsigned int col);

void duAppendCross(struct duDebugDraw* dd, const float x, const float y, const float z,
				   const float size, unsigned int col);

void duAppendBox(struct duDebugDraw* dd, float minx, float miny, float minz,
				 float maxx, float maxy, float maxz, const unsigned int* fcol);


class duDisplayList : public duDebugDraw
{
	float* m_pos;
	unsigned int* m_color;
	int m_size;
	int m_cap;

	bool m_depthMask;
	duDebugDrawPrimitives m_prim;
	float m_primSize;
	
	void resize(int cap);
	
public:
	duDisplayList(int cap = 512);
	~duDisplayList();
	virtual void depthMask(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void end();
	void clear();
	void draw(struct duDebugDraw* dd);
};


#endif // DEBUGDRAW_H
