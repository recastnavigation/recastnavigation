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

#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum GfxCmdType
{
	GFXCMD_RECT,
	GFXCMD_TRIANGLE,
	GFXCMD_TEXT,
	GFXCMD_SCISSOR,
};

struct GfxRect
{
	short x,y,w,h,r;
};
struct GfxText
{
	short x,y,dir;
	const char* text;
};

struct GfxCmd
{
	char type;
	char flags;
	char pad[2];
	unsigned int col;
	union
	{
		GfxRect rect;
		GfxText text;
	};
};

unsigned int RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	return (r) | (g << 8) | (b << 16) | (a << 24);
}

static const unsigned TEXT_POOL_SIZE = 4096;
static char g_textPool[TEXT_POOL_SIZE];
static unsigned g_textPoolSize = 0;
const char* allocText(const char* text)
{
	unsigned len = strlen(text)+1;
	if (g_textPoolSize + len >= TEXT_POOL_SIZE)
		return 0;
	char* dst = &g_textPool[g_textPoolSize]; 
	memcpy(dst, text, len);
	g_textPoolSize += len;
	return dst;
}

static const unsigned GFXCMD_QUEUE_SIZE = 1024;
static GfxCmd g_gfxCmdQueue[GFXCMD_QUEUE_SIZE];
static unsigned g_gfxCmdQueueSize = 0;

void resetGfxCmdQueue()
{
	g_gfxCmdQueueSize = 0;
	g_textPoolSize = 0;
}


static const unsigned TEMP_COORD_COUNT = 100;
static float g_tempCoords[TEMP_COORD_COUNT*2];
static float g_tempNormals[TEMP_COORD_COUNT*2];

static void drawPolygon(const float* coords, unsigned numCoords, float r, unsigned int col)
{
	if (numCoords > TEMP_COORD_COUNT) numCoords = TEMP_COORD_COUNT;
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		const float* v0 = &coords[j*2];
		const float* v1 = &coords[i*2];
		float dx = v1[0] - v0[0];
		float dy = v1[1] - v0[1];
		float d = sqrtf(dx*dx+dy*dy);
		if (d > 0)
		{
			d = 1.0f/d;
			dx *= d;
			dy *= d;
		}
		g_tempNormals[j*2+0] = dy;
		g_tempNormals[j*2+1] = -dx;
	}
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		float dlx0 = g_tempNormals[j*2+0];
		float dly0 = g_tempNormals[j*2+1];
		float dlx1 = g_tempNormals[i*2+0];
		float dly1 = g_tempNormals[i*2+1];
		float dmx = (dlx0 + dlx1) * 0.5f;
		float dmy = (dly0 + dly1) * 0.5f;
		float	dmr2 = dmx*dmx + dmy*dmy;
		if (dmr2 > 0.000001f)
		{
			float	scale = 1.0f / dmr2;
			if (scale > 10.0f) scale = 10.0f;
			dmx *= scale;
			dmy *= scale;
		}
		g_tempCoords[i*2+0] = coords[i*2+0]+dmx*r;
		g_tempCoords[i*2+1] = coords[i*2+1]+dmy*r;
	}
	
	unsigned int colTrans = RGBA(col&0xff, (col>>8)&0xff, (col>>16)&0xff, 0);
	
	glBegin(GL_TRIANGLES);
	
	glColor4ubv((GLubyte*)&col);
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		glVertex2fv(&coords[i*2]);
		glVertex2fv(&coords[j*2]);
		glColor4ubv((GLubyte*)&colTrans);
		glVertex2fv(&g_tempCoords[j*2]);
		
		glVertex2fv(&g_tempCoords[j*2]);
		glVertex2fv(&g_tempCoords[i*2]);
		
		glColor4ubv((GLubyte*)&col);
		glVertex2fv(&coords[i*2]);
	}
	
	glColor4ubv((GLubyte*)&col);
	for (unsigned i = 2; i < numCoords; ++i)
	{
		glVertex2fv(&coords[0]);
		glVertex2fv(&coords[(i-1)*2]);
		glVertex2fv(&coords[i*2]);
	}
	
	glEnd();
}

static const int CIRCLE_VERTS = 8*4;
static float g_circleVerts[CIRCLE_VERTS*2];
static bool g_circleVertsInitialized = false;

const float* getCircleVerts()
{
	if (!g_circleVertsInitialized)
	{
		g_circleVertsInitialized = true;
		for (unsigned i = 0; i < CIRCLE_VERTS; ++i)
		{
			float a = (float)i/(float)CIRCLE_VERTS * (float)M_PI*2;
			g_circleVerts[i*2+0] = cosf(a);
			g_circleVerts[i*2+1] = sinf(a);
		}
	}
	return g_circleVerts;
}

static void drawRect(float x, float y, float w, float h, float fth, unsigned int col)
{
	float verts[4*2] =
	{
		x, y,
		x+w, y,
		x+w, y+h,
		x, y+h,
	};
	drawPolygon(verts, 4, fth, col);
}

static void drawEllipse(float x, float y, float w, float h, float fth, unsigned int col)
{
	float verts[CIRCLE_VERTS*2];
	const float* cverts = getCircleVerts();
	float* v = verts;
	
	for (unsigned i = 0; i < CIRCLE_VERTS; ++i)
	{
		*v++ = x + cverts[i*2]*w;
		*v++ = y + cverts[i*2+1]*h;
	}
	
	drawPolygon(verts, CIRCLE_VERTS, fth, col);
}

static void drawRoundedRect(float x, float y, float w, float h, float r, float fth, unsigned int col)
{
	const unsigned n = CIRCLE_VERTS/4;
	float verts[(n+1)*4*2];
	const float* cverts = getCircleVerts();
	float* v = verts;
	
	for (unsigned i = 0; i <= n; ++i)
	{
		*v++ = x+w-r + cverts[i*2]*r;
		*v++ = y+h-r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n; i <= n*2; ++i)
	{
		*v++ = x+r + cverts[i*2]*r;
		*v++ = y+h-r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n*2; i <= n*3; ++i)
	{
		*v++ = x+r + cverts[i*2]*r;
		*v++ = y+r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n*3; i < n*4; ++i)
	{
		*v++ = x+w-r + cverts[i*2]*r;
		*v++ = y+r + cverts[i*2+1]*r;
	}
	*v++ = x+w-r + cverts[0]*r;
	*v++ = y+r + cverts[1]*r;
	
	drawPolygon(verts, (n+1)*4, fth, col);
}

static void drawLine(float x0, float y0, float x1, float y1, float r, float fth, unsigned int col)
{
	float dx = x1-x0;
	float dy = y1-y0;
	float d = sqrtf(dx*dx+dy*dy);
	if (d > 0.0001f)
	{
		d = 1.0f/d;
		dx *= d;
		dy *= d;
	}
	float t = dx;
	dx = dy;
	dy = -t;
	float verts[4*2];
	r -= fth;
	r *= 0.5f;
	if (r < 0.01f) r = 0.01f;
	dx *= r;
	dy *= r;
	
	verts[0] = x0-dx;
	verts[1] = y0-dy;
	
	verts[2] = x0+dx;
	verts[3] = y0+dy;
	
	verts[4] = x1+dx;
	verts[5] = y1+dy;
	
	verts[6] = x1-dx;
	verts[7] = y1-dy;
	
	drawPolygon(verts, 4, fth, col);
}


void renderGfxCmdQueue(void (*drawText)(int x, int y, int dir, const char* text, unsigned int col))
{
	glDisable(GL_SCISSOR_TEST);
	for (unsigned i = 0; i < g_gfxCmdQueueSize; ++i)
	{
		const GfxCmd& cmd = g_gfxCmdQueue[i];
		if (cmd.type == GFXCMD_RECT)
		{
			if (cmd.rect.r == 0)
			{
				drawRect((float)cmd.rect.x+0.5f, (float)cmd.rect.y+0.5f,
						 (float)cmd.rect.w-1, (float)cmd.rect.h-1,
						 1.0f, cmd.col);
			}
			else
			{
				drawRoundedRect((float)cmd.rect.x+0.5f, (float)cmd.rect.y+0.5f,
								(float)cmd.rect.w-1, (float)cmd.rect.h-1,
								(float)cmd.rect.r, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == GFXCMD_TRIANGLE)
		{
			glColor4ub(cmd.col&0xff, (cmd.col>>8)&0xff, (cmd.col>>16)&0xff, (cmd.col>>24)&0xff);
			if (cmd.flags == 1)
			{
				const float verts[3*2] =
				{
					(float)cmd.rect.x+0.5f, (float)cmd.rect.y+0.5f,
					(float)cmd.rect.x+0.5f+(float)cmd.rect.w-1, (float)cmd.rect.y+0.5f+(float)cmd.rect.h/2-0.5f,
					(float)cmd.rect.x+0.5f, (float)cmd.rect.y+0.5f+(float)cmd.rect.h-1,
				};
				drawPolygon(verts, 3, 1.0f, cmd.col);
			}
			if (cmd.flags == 2)
			{
				const float verts[3*2] =
				{
					(float)cmd.rect.x+0.5f, (float)cmd.rect.y+(float)cmd.rect.h-1,
					(float)cmd.rect.x+0.5f+(float)cmd.rect.w/2-0.5f, (float)cmd.rect.y+0.5f,
					(float)cmd.rect.x+0.5f+(float)cmd.rect.w-1, (float)cmd.rect.y+0.5f+(float)cmd.rect.h-1,
				};
				drawPolygon(verts, 3, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == GFXCMD_TEXT)
		{
			drawText(cmd.text.x, cmd.text.y, cmd.text.dir, cmd.text.text, cmd.col);
		}
		else if (cmd.type == GFXCMD_SCISSOR)
		{
			if (cmd.flags)
			{
				glEnable(GL_SCISSOR_TEST);
				glScissor(cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h);
			}
			else
			{
				glDisable(GL_SCISSOR_TEST);
			}
		}
	}
	glDisable(GL_SCISSOR_TEST);
}

void addGfxCmdScissor(int x, int y, int w, int h)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_SCISSOR;
	cmd.flags = x < 0 ? 0 : 1;	// on/off flag.
	cmd.col = 0;
	cmd.rect.x = (short)x;
	cmd.rect.y = (short)y;
	cmd.rect.w = (short)w;
	cmd.rect.h = (short)h;
}

void addGfxCmdRect(int x, int y, int w, int h, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_RECT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.rect.x = (short)x;
	cmd.rect.y = (short)y;
	cmd.rect.w = (short)w;
	cmd.rect.h = (short)h;
	cmd.rect.r = 0;
}

void addGfxCmdRoundedRect(int x, int y, int w, int h, int r, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_RECT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.rect.x = (short)x;
	cmd.rect.y = (short)y;
	cmd.rect.w = (short)w;
	cmd.rect.h = (short)h;
	cmd.rect.r = (short)r;
}

void addGfxCmdTriangle(int x, int y, int w, int h, int flags, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_TRIANGLE;
	cmd.flags = (char)flags;
	cmd.col = color;
	cmd.rect.x = (short)x;
	cmd.rect.y = (short)y;
	cmd.rect.w = (short)w;
	cmd.rect.h = (short)h;
}

void addGfxCmdText(int x, int y, int dir, const char* text, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_TEXT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.text.x = (short)x;
	cmd.text.y = (short)y;
	cmd.text.dir = (short)dir;
	cmd.text.text = allocText(text);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GuiState
{
	GuiState() :
		mbutPressed(false), mbutReleased(false), mbut(false), mx(-1), my(-1),
		isHot(false), isActive(false), wentActive(false),
		dragX(0), dragY(0), dragOrig(0),
		widgetX(0), widgetY(0), widgetW(100),
		active(0), hot(0), hotToBe(0)
	{
	}

	bool mbutPressed, mbutReleased;
	bool mbut;
	int mx,my;
	unsigned int active;
	unsigned int hot;
	unsigned int hotToBe;
	bool isHot;
	bool isActive;
	bool wentActive;
	int dragX, dragY;
	float dragOrig;
	int widgetX, widgetY, widgetW;
};

static GuiState g_state;

inline bool anyActive()
{
	return g_state.active != 0;
}

inline bool isActive(unsigned int id)
{
	return g_state.active == id;
}

inline bool isHot(unsigned int id)
{
	return g_state.hot == id;
}

inline bool inRect(int x, int y, int w, int h)
{
   return g_state.mx >= x && g_state.mx <= x+w && g_state.my >= y && g_state.my <= y+h;
}

void clearInput()
{
	g_state.mbutPressed = false;
	g_state.mbutReleased = false;
}

void clearActive(void)
{
	g_state.active = 0;
	// mark all UI for this frame as processed
	clearInput();
}

void setActive(unsigned int id)
{
	g_state.active = id;
	g_state.wentActive = true;
}

void setHot(unsigned int id)
{
   g_state.hotToBe = id;
}


bool buttonLogic(unsigned int id, bool over)
{
	bool res = false;
	// process down
	if (!anyActive())
	{
		if (over)
			setHot(id);
		if (isHot(id) && g_state.mbutPressed)
			setActive(id);
	}

	// if button is active, then react on left up
	if (isActive(id))
	{
		g_state.isActive = true;
		if (over)
			setHot(id);
		if (g_state.mbutReleased)
		{
			if (isHot(id))
				res = true;
			clearActive();
		}
	}

	if (isHot(id))
		g_state.isHot = true;

	return res;
}

static void updateInput()
{
	int mx, my;
	Uint8 state = SDL_GetMouseState(&mx, &my);
	bool mbut = (state & SDL_BUTTON_LMASK) != 0;
	SDL_Surface* screen = SDL_GetVideoSurface();
	my = screen->h-1 - my;

	g_state.mx = mx;
	g_state.my = my;
	g_state.mbutPressed = !g_state.mbut && mbut;
	g_state.mbutReleased = g_state.mbut && !mbut;
	g_state.mbut = mbut;
}

void imguiBeginFrame()
{
	updateInput();

	g_state.hot = g_state.hotToBe;
	g_state.hotToBe = 0;

	g_state.wentActive = false;
	g_state.isActive = false;
	g_state.isHot = false;

	g_state.widgetX = 0;
	g_state.widgetY = 0;
	g_state.widgetW = 0;

	resetGfxCmdQueue();
}

void imguiEndFrame()
{
	clearInput();
}

void imguiRender(void (*drawText)(int x, int y, int dir, const char* text, unsigned int col))
{
   renderGfxCmdQueue(drawText);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int BUTTON_HEIGHT = 20;
static const int SLIDER_HEIGHT = 20;
static const int SLIDER_MARKER_WIDTH = 10;
static const int CHECK_SIZE = 8;
static const int DEFAULT_SPACING = 4;
static const int TEXT_HEIGHT = 8;
static const int SCROLL_AREA_PADDING = 6;
static const int INTEND_SIZE = 16;
static const int AREA_HEADER = 28;

static int g_scrollTop = 0;
static int g_scrollBottom = 0;
static int g_scrollRight = 0;
static int g_scrollAreaTop = 0;
static int* g_scrollVal = 0;
static int g_focusTop = 0;
static int g_focusBottom = 0;
static unsigned int g_scrollId = 0; 

bool imguiBeginScrollArea(unsigned int id, const char* name, int x, int y, int w, int h, int* scroll)
{
	g_scrollId = id;

	g_state.widgetX = x + SCROLL_AREA_PADDING;
	g_state.widgetY = y+h-AREA_HEADER + (*scroll);
	g_state.widgetW = w - SCROLL_AREA_PADDING*4;
	g_scrollTop = y-AREA_HEADER+h;
	g_scrollBottom = y+SCROLL_AREA_PADDING;
	g_scrollRight = x+w - SCROLL_AREA_PADDING*3;
	g_scrollVal = scroll;

	g_scrollAreaTop = g_state.widgetY;

	g_focusTop = y-AREA_HEADER;
	g_focusBottom = y-AREA_HEADER+h;

	addGfxCmdRoundedRect(x, y, w, h, 6, RGBA(0,0,0,192));

	addGfxCmdText(x+AREA_HEADER/2, y+h-AREA_HEADER/2-TEXT_HEIGHT/2, 1, name, RGBA(255,255,255,128));

	addGfxCmdScissor(x+SCROLL_AREA_PADDING, y+SCROLL_AREA_PADDING, w-SCROLL_AREA_PADDING*4, h-AREA_HEADER-SCROLL_AREA_PADDING);

	return inRect(x, y, w, h);
}

void imguiEndScrollArea()
{
	// Disable scissoring.
	addGfxCmdScissor(-1,-1,-1,-1);

	// Draw scroll bar
	int x = g_scrollRight+SCROLL_AREA_PADDING/2;
	int y = g_scrollBottom;
	int w = SCROLL_AREA_PADDING*2;
	int h = g_scrollTop - g_scrollBottom;

	int stop = g_scrollAreaTop;
	int sbot = g_state.widgetY;
	int sh = stop - sbot; // The scrollable area height.

	float barHeight = (float)h/(float)sh;
	
	if (barHeight < 1)
	{
		float barY = (float)(y - sbot)/(float)sh;
		if (barY < 0) barY = 0;
		if (barY > 1) barY = 1;
		
		// Handle scroll bar logic.
		unsigned int hid = g_scrollId;
		int hx = x;
		int hy = y + (int)(barY*h);
		int hw = w;
		int hh = (int)(barHeight*h);
		
		const int range = h - (hh-1);
		bool over = inRect(hx, hy, hw, hh);
		buttonLogic(hid, over);
		if (isActive(hid))
		{
			float u = (float)(hy-y) / (float)range;
			if (g_state.wentActive)
			{
				g_state.dragY = g_state.my;
				g_state.dragOrig = u;
			}
			if (g_state.dragY != g_state.my)
			{
				u = g_state.dragOrig + (g_state.my - g_state.dragY) / (float)range;
				if (u < 0) u = 0;
				if (u > 1) u = 1;
				*g_scrollVal = (int)((1-u) * (sh - h));
			}
		}
		
		// BG
		addGfxCmdRoundedRect(x, y, w, h, w/2-1, RGBA(0,0,0,196));
		// Bar
		if (isActive(hid))
			addGfxCmdRoundedRect(hx, hy, hw, hh, w/2-1, RGBA(255,196,0,196));
		else
			addGfxCmdRoundedRect(hx, hy, hw, hh, w/2-1, isHot(hid) ? RGBA(255,196,0,96) : RGBA(255,255,255,64));
	}
}

bool imguiButton(unsigned int id, const char* text)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	int w = g_state.widgetW;
	int h = BUTTON_HEIGHT;
	g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

	bool over = inRect(x, y, w, h);
	bool res = buttonLogic(id, over);

	addGfxCmdRoundedRect(x, y, w, h, BUTTON_HEIGHT/2-1, RGBA(128,128,128, isActive(id)?196:96));
	addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, 1, text, isHot(id) ? RGBA(255,196,0,255) : RGBA(255,255,255,200));

	return res;
}

bool imguiItem(unsigned int id, const char* text)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	int w = g_state.widgetW;
	int h = BUTTON_HEIGHT;
	g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;
	
	bool over = inRect(x, y, w, h);
	bool res = buttonLogic(id, over);
	
	if (isHot(id))
		addGfxCmdRoundedRect(x, y, w, h, 2, RGBA(255,196,0,isActive(id)?196:96));
	addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, 1, text, RGBA(255,255,255,200));
	
	return res;
}

bool imguiCheck(unsigned int id, const char* text, bool checked)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	int w = g_state.widgetW;
	int h = BUTTON_HEIGHT;
	g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

	bool over = inRect(x, y, w, h);
	bool res = buttonLogic(id, over);
	
	const int cx = x+BUTTON_HEIGHT/2-CHECK_SIZE/2;
	const int cy = y+BUTTON_HEIGHT/2-CHECK_SIZE/2;
	addGfxCmdRoundedRect(cx-3, cy-3, CHECK_SIZE+6, CHECK_SIZE+6, 4, RGBA(128,128,128, isActive(id)?196:96));
	if (checked)
		addGfxCmdRoundedRect(cx, cy, CHECK_SIZE, CHECK_SIZE, CHECK_SIZE/2-1, RGBA(255,255,255,isActive(id)?255:200));

	addGfxCmdText(x+BUTTON_HEIGHT, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, 1, text, isHot(id) ? RGBA(255,196,0,255) : RGBA(255,255,255,200));

	return res;
}

bool imguiCollapse(unsigned int id, const char* text, bool checked)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	int w = g_state.widgetW;
	int h = BUTTON_HEIGHT;
	g_state.widgetY -= BUTTON_HEIGHT; // + DEFAULT_SPACING;

	const int cx = x+BUTTON_HEIGHT/2-CHECK_SIZE/2;
	const int cy = y+BUTTON_HEIGHT/2-CHECK_SIZE/2;

	bool over = inRect(x, y, w, h);
	bool res = buttonLogic(id, over);
	
	if (checked)
		addGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 1, RGBA(255,255,255,isActive(id)?255:200));
	else
		addGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 2, RGBA(255,255,255,isActive(id)?255:200));

	addGfxCmdText(x+BUTTON_HEIGHT, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, 1, text, isHot(id) ? RGBA(255,196,0,255) : RGBA(255,255,255,200));

	return res;
}

void imguiLabel(unsigned int /*id*/, const char* text)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	g_state.widgetY -= BUTTON_HEIGHT;
	addGfxCmdText(x, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, 1, text, RGBA(255,255,255,255));
}

void imguiValue(unsigned int /*id*/, const char* text)
{
	const int x = g_state.widgetX;
	const int y = g_state.widgetY - BUTTON_HEIGHT;
	const int w = g_state.widgetW;
	g_state.widgetY -= BUTTON_HEIGHT;
	
	addGfxCmdText(x+w-BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, -1, text, RGBA(255,255,255,200));
}

bool imguiSlider(unsigned int id, const char* text, float* val, float vmin, float vmax, float vinc)
{
	int x = g_state.widgetX;
	int y = g_state.widgetY - BUTTON_HEIGHT;
	int w = g_state.widgetW;
	int h = SLIDER_HEIGHT;
	g_state.widgetY -= SLIDER_HEIGHT + DEFAULT_SPACING;

	addGfxCmdRoundedRect(x, y, w, h, 4, RGBA(0,0,0,128));

	const int range = w - SLIDER_MARKER_WIDTH;

	float u = (*val - vmin) / (vmax-vmin);
	if (u < 0) u = 0;
	if (u > 1) u = 1;
	int m = (int)(u * range);

	bool over = inRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT);
	bool res = buttonLogic(id, over);
	bool valChanged = false;

	if (isActive(id))
	{
		if (g_state.wentActive)
		{
			g_state.dragX = g_state.mx;
			g_state.dragOrig = u;
		}
		if (g_state.dragX != g_state.mx)
		{
			u = g_state.dragOrig + (float)(g_state.mx - g_state.dragX) / (float)range;
			if (u < 0) u = 0;
			if (u > 1) u = 1;
			*val = vmin + u*(vmax-vmin);
			*val = floorf(*val / vinc)*vinc; // Snap to vinc
			m = (int)(u * range);
			valChanged = true;
		}
	}

	if (isActive(id))
		addGfxCmdRoundedRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT, 4, RGBA(255,255,255,255));
	else
		addGfxCmdRoundedRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT, 4, isHot(id) ? RGBA(255,196,0,128) : RGBA(255,255,255,64));

	int digits = (int)(ceilf(log10f(vinc)));
	char fmt[16];
	snprintf(fmt, 16, "%%.%df", digits >= 0 ? 0 : -digits);
	char msg[128];
	snprintf(msg, 128, fmt, *val);
	
	addGfxCmdText(x+SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, 1, text, isHot(id) ? RGBA(255,196,0,255) : RGBA(255,255,255,200));
	addGfxCmdText(x+w-SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, -1, msg, isHot(id) ? RGBA(255,196,0,255) : RGBA(255,255,255,200));

	return res || valChanged;
}


void imguiIndent()
{
	g_state.widgetX += INTEND_SIZE;
	g_state.widgetW -= INTEND_SIZE;
}

void imguiUnindent()
{
	g_state.widgetX -= INTEND_SIZE;
	g_state.widgetW += INTEND_SIZE;
}

void imguiSeparator()
{
	g_state.widgetY -= DEFAULT_SPACING*3;
}