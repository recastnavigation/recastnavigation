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

#pragma once
#include <cstdint>

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
	virtual ~duDebugDraw() = 0;
	
	virtual void depthMask(bool state) = 0;

	virtual void texture(bool state) = 0;

	/// Begin drawing primitives.
	///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
	///  @param size [in] size of a primitive, applies to point size and line width only.
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float* pos, uint32_t color) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(float x, float y, float z, uint32_t color) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param uv [in] the uv coordinates of the verts.
	virtual void vertex(const float* pos, uint32_t color, const float* uv) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param u,v [in] the uv coordinates of the verts.
	virtual void vertex(float x, float y, float z, uint32_t color, float u, float v) = 0;

	/// End drawing primitives.
	virtual void end() = 0;

	/// Compute a color for given area.
	virtual uint32_t areaToCol(uint32_t area);
};

inline uint32_t duRGBA(const int r, const int g, const int b, const int a)
{
	return static_cast<uint32_t>(r) | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) << 16 | static_cast<uint32_t>(a) << 24;
}

inline uint32_t duRGBAf(const float fr, const float fg, const float fb, const float fa)
{
	const uint8_t r = static_cast<uint8_t>(fr * 255.0f);
	const uint8_t g = static_cast<uint8_t>(fg * 255.0f);
	const uint8_t b = static_cast<uint8_t>(fb * 255.0f);
	const uint8_t a = static_cast<uint8_t>(fa * 255.0f);
	return duRGBA(r,g,b,a);
}

uint32_t duIntToCol(int i, int a);
void duIntToCol(int i, float* col);

inline uint32_t duMultCol(const uint32_t col, const uint32_t d)
{
	const uint32_t r = col & 0xff;
	const uint32_t g = col >> 8 & 0xff;
	const uint32_t b = col >> 16 & 0xff;
	const uint32_t a = col >> 24 & 0xff;
	return duRGBA(r*d >> 8, g*d >> 8, b*d >> 8, a);
}

inline uint32_t duDarkenCol(const uint32_t col)
{
	return col >> 1 & 0x007f7f7f | col & 0xff000000;
}

inline uint32_t duLerpCol(const uint32_t ca, const uint32_t cb, const uint32_t u)
{
	const uint32_t ra = ca & 0xff;
	const uint32_t ga = ca >> 8 & 0xff;
	const uint32_t ba = ca >> 16 & 0xff;
	const uint32_t aa = ca >> 24 & 0xff;
	const uint32_t rb = cb & 0xff;
	const uint32_t gb = cb >> 8 & 0xff;
	const uint32_t bb = cb >> 16 & 0xff;
	const uint32_t ab = cb >> 24 & 0xff;

	const uint32_t r = (ra*(255-u) + rb*u)/255;
	const uint32_t g = (ga*(255-u) + gb*u)/255;
	const uint32_t b = (ba*(255-u) + bb*u)/255;
	const uint32_t a = (aa*(255-u) + ab*u)/255;
	return duRGBA(r,g,b,a);
}

inline uint32_t duTransCol(const uint32_t c, const uint32_t a)
{
	return a<<24 | c & 0x00ffffff;
}


void duCalcBoxColors(uint32_t* colors, uint32_t colTop, uint32_t colSide);

void duDebugDrawCylinderWire(duDebugDraw* dd, float minx, float miny, float minz,
                             float maxx, float maxy, float maxz, uint32_t col, float lineWidth);

void duDebugDrawBoxWire(duDebugDraw* dd, float minx, float miny, float minz,
                        float maxx, float maxy, float maxz, uint32_t col, float lineWidth);

void duDebugDrawArc(duDebugDraw* dd, float x0, float y0, float z0,
                    float x1, float y1, float z1, float h,
                    float as0, float as1, uint32_t col, float lineWidth);

void duDebugDrawArrow(duDebugDraw* dd, float x0, float y0, float z0,
                      float x1, float y1, float z1,
                      float as0, float as1, uint32_t col, float lineWidth);

void duDebugDrawCircle(duDebugDraw* dd, float x, float y, float z,
                       float r, uint32_t col, float lineWidth);

void duDebugDrawCross(duDebugDraw* dd, float x, float y, float z,
                      float size, uint32_t col, float lineWidth);

void duDebugDrawBox(duDebugDraw* dd, float minx, float miny, float minz,
                    float maxx, float maxy, float maxz, const uint32_t* fcol);

void duDebugDrawCylinder(duDebugDraw* dd, float minx, float miny, float minz,
                         float maxx, float maxy, float maxz, uint32_t col);

void duDebugDrawGridXZ(duDebugDraw* dd, float ox, float oy, float oz,
                       int w, int h, float size,
                       uint32_t col, float lineWidth);


// Versions without begin/end, can be used to draw multiple primitives.
void duAppendCylinderWire(duDebugDraw* dd, float minx, float miny, float minz,
                          float maxx, float maxy, float maxz, uint32_t col);

void duAppendBoxWire(duDebugDraw* dd, float minx, float miny, float minz,
                     float maxx, float maxy, float maxz, uint32_t col);

void duAppendBoxPoints(duDebugDraw* dd, float minx, float miny, float minz,
                       float maxx, float maxy, float maxz, uint32_t col);

void duAppendArc(duDebugDraw* dd, float x0, float y0, float z0,
                 float x1, float y1, float z1, float h,
                 float as0, float as1, uint32_t col);

void duAppendArrow(duDebugDraw* dd, float x0, float y0, float z0,
                   float x1, float y1, float z1,
                   float as0, float as1, uint32_t col);

void duAppendCircle(duDebugDraw* dd, float x, float y, float z,
                    float r, uint32_t col);

void duAppendCross(duDebugDraw* dd, float x, float y, float z,
                   float size, uint32_t col);

void duAppendBox(duDebugDraw* dd, float minx, float miny, float minz,
                 float maxx, float maxy, float maxz, const uint32_t* fcol);

void duAppendCylinder(duDebugDraw* dd, float minx, float miny, float minz,
                      float maxx, float maxy, float maxz, uint32_t col);


class duDisplayList : public duDebugDraw
{
	float* m_pos{};
	uint32_t* m_color{};
	int m_size{};
	int m_cap{};

	duDebugDrawPrimitives m_prim{DU_DRAW_LINES};
	float m_primSize{1.0f};
	bool m_depthMask{true};

	void resize(int cap);

public:
	explicit duDisplayList(int cap = 512);
	~duDisplayList() override;
	void depthMask(bool state) override;
	void begin(duDebugDrawPrimitives prim, float size = 1.0f) override;
	void vertex(float x, float y, float z, uint32_t color) override;
	void vertex(const float* pos, uint32_t color) override;
	void end() override;
	void clear();
	void draw(duDebugDraw* dd) const;

	// Explicitly disabled copy constructor and copy assignment operator.
	duDisplayList(const duDisplayList&) = delete;
	duDisplayList& operator=(const duDisplayList&) = delete;
};
