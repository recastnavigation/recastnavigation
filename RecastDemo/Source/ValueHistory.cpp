#include "ValueHistory.h"
#include "imgui.h"
#include <string.h>
#include <stdio.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

ValueHistory::ValueHistory() :
	m_hsamples(0)
{
	for (int i = 0; i < MAX_HISTORY; ++i)
		m_samples[i] = 0;
}

ValueHistory::~ValueHistory()
{
}

float ValueHistory::getSampleMin() const
{
	float val = m_samples[0];
	for (int i = 1; i < MAX_HISTORY; ++i)
		if (m_samples[i] < val)
			val = m_samples[i];
	return val;
} 

float ValueHistory::getSampleMax() const
{
	float val = m_samples[0];
	for (int i = 1; i < MAX_HISTORY; ++i)
		if (m_samples[i] > val)
			val = m_samples[i];
	return val;
}

float ValueHistory::getAverage() const
{
	float val = 0;
	for (int i = 0; i < MAX_HISTORY; ++i)
		val += m_samples[i];
	return val/(float)MAX_HISTORY;
}

void GraphParams::setRect(int ix, int iy, int iw, int ih, int ipad)
{
	x = ix;
	y = iy;
	w = iw;
	h = ih;
	pad = ipad;
}

void GraphParams::setValueRange(float ivmin, float ivmax, int indiv, const char* iunits)
{
	vmin = ivmin;
	vmax = ivmax;
	ndiv = indiv;
	strcpy(units, iunits);
}

void drawGraphBackground(const GraphParams* p)
{
	// BG
	imguiDrawRoundedRect((float)p->x, (float)p->y, (float)p->w, (float)p->h, (float)p->pad, imguiRGBA(64,64,64,128));
	
	const float sy = (p->h-p->pad*2) / (p->vmax-p->vmin);
	const float oy = p->y+p->pad-p->vmin*sy;
	
	char text[64];
	
	// Divider Lines
	for (int i = 0; i <= p->ndiv; ++i)
	{
		const float u = (float)i/(float)p->ndiv;
		const float v = p->vmin + (p->vmax-p->vmin)*u;
		snprintf(text, 64, "%.2f %s", v, p->units);
		const float fy = oy + v*sy;
		imguiDrawText(p->x + p->w - p->pad, (int)fy-4, IMGUI_ALIGN_RIGHT, text, imguiRGBA(0,0,0,255));
		imguiDrawLine((float)p->x + (float)p->pad, fy, (float)p->x + (float)p->w - (float)p->pad - 50, fy, 1.0f, imguiRGBA(0,0,0,64)); 
	}
}

void drawGraph(const GraphParams* p, const ValueHistory* graph,
			   int idx, const char* label, const unsigned int col)
{
	const float sx = (p->w - p->pad*2) / (float)graph->getSampleCount();
	const float sy = (p->h - p->pad*2) / (p->vmax - p->vmin);
	const float ox = (float)p->x + (float)p->pad;
	const float oy = (float)p->y + (float)p->pad - p->vmin*sy;
	
	// Values
	float px=0, py=0;
	for (int i = 0; i < graph->getSampleCount()-1; ++i)
	{
		const float x = ox + i*sx;
		const float y = oy + graph->getSample(i)*sy;
		if (i > 0)
			imguiDrawLine(px,py, x,y, 2.0f, col);
		px = x;
		py = y;
	}
	
	// Label
	const int size = 15;
	const int spacing = 10;
	int ix = p->x + p->w + 5;
	int iy = p->y + p->h - (idx+1)*(size+spacing);
	
	imguiDrawRoundedRect((float)ix, (float)iy, (float)size, (float)size, 2.0f, col);
	
	char text[64];
	snprintf(text, 64, "%.2f %s", graph->getAverage(), p->units);
	imguiDrawText(ix+size+5, iy+3, IMGUI_ALIGN_LEFT, label, imguiRGBA(255,255,255,192));
	imguiDrawText(ix+size+150, iy+3, IMGUI_ALIGN_RIGHT, text, imguiRGBA(255,255,255,128));
}

