#include "ValueHistory.h"

#include "imgui.h"

#include <cstdio>
#include <cstring>

#ifdef WIN32
#	define snprintf _snprintf
#endif

float ValueHistory::getSampleMin() const
{
	float min = samples[0];
	for (int sample : samples)
	{
		if (sample < min)
		{
			min = sample;
		}
	}
	return min;
}

float ValueHistory::getSampleMax() const
{
	float max = samples[0];
	for (int sample : samples)
	{
		if (sample > max)
		{
			max = sample;
		}
	}
	return max;
}

float ValueHistory::getAverage() const
{
	float total = 0;
	for (int sample : samples)
	{
		total += sample;
	}
	return total / samples.size();
}

void GraphParams::setRect(int x, int y, int width, int height, int padding)
{
	this->x = x;
	this->y = y;
	this->width = width;
	this->height = height;
	this->padding = padding;
}

void GraphParams::setValueRange(float minValue, float maxValue, int numDivisions, const std::string& units)
{
	this->rangeMin = minValue;
	this->rangeMax = maxValue;
	this->rangeDivisions = numDivisions;
	this->units = units;
}

void drawGraphBackground(const GraphParams* params)
{
	// BG
	imguiDrawRoundedRect(
		(float)params->x,
		(float)params->y,
		(float)params->width,
		(float)params->height,
		(float)params->padding,
		imguiRGBA(64, 64, 64, 128));

	const float pixelScaleY = (params->height - params->padding * 2) / (params->rangeMax - params->rangeMin);
	const float offsetY = params->y + params->padding - params->rangeMin * pixelScaleY;

	char valueLabel[64];

	// Divider Lines
	for (int divisionIndex = 0; divisionIndex <= params->rangeDivisions; ++divisionIndex)
	{
		const float normalizedPosition = (float)divisionIndex / (float)params->rangeDivisions;
		const float valueAtDivision = params->rangeMin + (params->rangeMax - params->rangeMin) * normalizedPosition;
		snprintf(valueLabel, 64, "%.2f %s", valueAtDivision, params->units.c_str());
		const float fy = offsetY + valueAtDivision * pixelScaleY;
		imguiDrawText(
			params->x + params->width - params->padding,
			(int)fy - 4,
			IMGUI_ALIGN_RIGHT,
			valueLabel,
			imguiRGBA(0, 0, 0, 255));
		imguiDrawLine(
			(float)params->x + (float)params->padding,
			fy,
			(float)params->x + (float)params->width - (float)params->padding - 50,
			fy,
			1.0f,
			imguiRGBA(0, 0, 0, 64));
	}
}

void drawGraph(const GraphParams* params, const ValueHistory* graph, int index, const char* label, const unsigned int color)
{
	const float sx = (params->width - params->padding * 2) / (float)graph->getSampleCount();
	const float sy = (params->height - params->padding * 2) / (params->rangeMax - params->rangeMin);
	const float ox = (float)params->x + (float)params->padding;
	const float oy = (float)params->y + (float)params->padding - params->rangeMin * sy;

	// Values
	float px = 0, py = 0;
	for (int i = 0; i < graph->getSampleCount() - 1; ++i)
	{
		const float x = ox + i * sx;
		const float y = oy + graph->getSample(i) * sy;
		if (i > 0)
		{
			imguiDrawLine(px, py, x, y, 2.0f, color);
		}
		px = x;
		py = y;
	}

	// Label
	const int size = 15;
	const int spacing = 10;
	int ix = params->x + params->width + 5;
	int iy = params->y + params->height - (index + 1) * (size + spacing);

	imguiDrawRoundedRect((float)ix, (float)iy, (float)size, (float)size, 2.0f, color);

	char text[64];
	snprintf(text, 64, "%.2f %s", graph->getAverage(), params->units.c_str());
	imguiDrawText(ix + size + 5, iy + 3, IMGUI_ALIGN_LEFT, label, imguiRGBA(255, 255, 255, 192));
	imguiDrawText(ix + size + 150, iy + 3, IMGUI_ALIGN_RIGHT, text, imguiRGBA(255, 255, 255, 128));
}
