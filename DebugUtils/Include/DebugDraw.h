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

// Some math headers don't have PI defined.
static constexpr float DU_PI = 3.14159265f;

enum duDebugDrawPrimitives
{
	DU_DRAW_POINTS,
	DU_DRAW_LINES,
	DU_DRAW_TRIS,
	DU_DRAW_QUADS
};

/// Abstract debug draw interface.
struct duDebugDraw
{
	virtual ~duDebugDraw() = default;
	
	virtual void depthMask(bool state) = 0;

	virtual void texture(bool state) = 0;

	/// Begin drawing primitives.
	///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
	///  @param size [in] size of a primitive, applies to point size and line width only.
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float* pos, unsigned int color) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(float x, float y, float z, unsigned int color) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param uv [in] the uv coordinates of the verts.
	virtual void vertex(const float* pos, unsigned int color, const float* uv) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param u,v [in] the uv coordinates of the verts.
	virtual void vertex(float x, float y, float z, unsigned int color, float u, float v) = 0;

	/// End drawing primitives.
	virtual void end() = 0;

	/// Compute a color for given area.
	virtual unsigned int areaToCol(unsigned int area);
};

inline unsigned int duRGBA(const int r, const int g, const int b, const int a)
{
	return static_cast<unsigned int>(r) | static_cast<unsigned int>(g) << 8 | static_cast<unsigned int>(b) << 16 | static_cast<unsigned int>(a) << 24;
}

inline unsigned int duRGBAf(const float fr, const float fg, const float fb, const float fa)
{
	const auto r = static_cast<unsigned char>(fr * 255.0f);
	const auto g = static_cast<unsigned char>(fg * 255.0f);
	const auto b = static_cast<unsigned char>(fb * 255.0f);
	const auto a = static_cast<unsigned char>(fa * 255.0f);
	return duRGBA(r,g,b,a);
}

unsigned int duIntToCol(int i, int a);
void duIntToCol(int i, float* col);

inline unsigned int duMultCol(const unsigned int col, const unsigned int d)
{
	const unsigned int r = col & 0xff;
	const unsigned int g = col >> 8 & 0xff;
	const unsigned int b = col >> 16 & 0xff;
	const unsigned int a = col >> 24 & 0xff;
	return duRGBA(static_cast<int>(r * d) >> 8, static_cast<int>(g * d) >> 8, static_cast<int>(b * d) >> 8, static_cast<int>(a));
}

inline unsigned int duDarkenCol(const unsigned int col)
{
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000);
}

inline unsigned int duLerpCol(const unsigned int ca, const unsigned int cb, const unsigned int u)
{
	const unsigned int ra = ca & 0xff;
	const unsigned int ga = ca >> 8 & 0xff;
	const unsigned int ba = ca >> 16 & 0xff;
	const unsigned int aa = ca >> 24 & 0xff;
	const unsigned int rb = cb & 0xff;
	const unsigned int gb = cb >> 8 & 0xff;
	const unsigned int bb = cb >> 16 & 0xff;
	const unsigned int ab = cb >> 24 & 0xff;

	const int r = static_cast<int>(ra * (255 - u) + rb * u)/255;
	const int g = static_cast<int>(ga * (255 - u) + gb * u)/255;
	const int b = static_cast<int>(ba * (255 - u) + bb * u)/255;
	const int a = static_cast<int>(aa * (255 - u) + ab * u)/255;
	return duRGBA(r,g,b,a);
}

inline unsigned int duTransCol(const unsigned int c, const unsigned int a)
{
	return (a<<24) | (c & 0x00ffffff);
}


void duCalcBoxColors(unsigned int* colors, unsigned int colTop, unsigned int colSide);

void duDebugDrawCylinderWire(duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, unsigned int col, float lineWidth);

void duDebugDrawBoxWire(duDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, unsigned int col, float lineWidth);

void duDebugDrawArc(duDebugDraw* dd, float x0, float y0, float z0,
                    float x1, float y1, float z1, float h,
                    float as0, float as1, unsigned int col, float lineWidth);

void duDebugDrawArrow(duDebugDraw* dd, float x0, float y0, float z0,
                      float x1, float y1, float z1,
                      float as0, float as1, unsigned int col, float lineWidth);

void duDebugDrawCircle(duDebugDraw* dd, float x, float y, float z,
                       float r, unsigned int col, float lineWidth);

void duDebugDrawCross(duDebugDraw* dd, float x, float y, float z,
                      float size, unsigned int col, float lineWidth);

void duDebugDrawBox(duDebugDraw* dd, float minx, float miny, float minz,
                    float maxx, float maxy, float maxz, const unsigned int* fcol);

void duDebugDrawCylinder(duDebugDraw* dd, float minx, float miny, float minz,
                         float maxx, float maxy, float maxz, unsigned int col);

void duDebugDrawGridXZ(duDebugDraw* dd, float ox, float oy, float oz,
                       int w, int h, float size,
                       unsigned int col, float lineWidth);


// Versions without begin/end, can be used to draw multiple primitives.
void duAppendCylinderWire(duDebugDraw* dd, float minx, float miny, float minz,
                          float maxx, float maxy, float maxz, unsigned int col);

void duAppendBoxWire(duDebugDraw* dd, float minx, float miny, float minz,
                     float maxx, float maxy, float maxz, unsigned int col);

void duAppendBoxPoints(duDebugDraw* dd, float minx, float miny, float minz,
                       float maxx, float maxy, float maxz, unsigned int col);

void duAppendArc(duDebugDraw* dd, float x0, float y0, float z0,
                 float x1, float y1, float z1, float h,
                 float as0, float as1, unsigned int col);

void duAppendArrow(duDebugDraw* dd, float x0, float y0, float z0,
                   float x1, float y1, float z1,
                   float as0, float as1, unsigned int col);

void duAppendCircle(duDebugDraw* dd, float x, float y, float z,
                    float r, unsigned int col);

void duAppendCross(duDebugDraw* dd, float x, float y, float z,
                   float size, unsigned int col);

void duAppendBox(duDebugDraw* dd, float minx, float miny, float minz,
                 float maxx, float maxy, float maxz, const unsigned int* fcol);

void duAppendCylinder(duDebugDraw* dd, float minx, float miny, float minz,
                      float maxx, float maxy, float maxz, unsigned int col);


class duDisplayList : public duDebugDraw
{
	float* m_pos;
	unsigned int* m_color;
	int m_size;
	int m_cap;

	duDebugDrawPrimitives m_prim;
	float m_primSize;
	bool m_depthMask;

	void resize(int cap);

public:
	explicit duDisplayList(int cap = 512);
	~duDisplayList() override;
	void depthMask(bool state) override;
	void begin(duDebugDrawPrimitives prim, float size = 1.0f) override;
	void vertex(float x, float y, float z, unsigned int color) override;
	void vertex(const float* pos, unsigned int color) override;
	void end() override;
	void clear();
	void draw(duDebugDraw* dd) const;
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	duDisplayList(const duDisplayList&);
	duDisplayList& operator=(const duDisplayList&);
};


#endif // DEBUGDRAW_H
