#pragma once

#include <string>
#include <vector>

class ValueHistory
{
	static constexpr int MAX_HISTORY = 256;
	std::vector<int> samples{};
	int nextSampleIndex = 0;

public:
	ValueHistory() { samples.reserve(MAX_HISTORY); }

	inline void addSample(const float val)
	{
		if (samples.size() < MAX_HISTORY)
		{
			samples.push_back(val);
		}
		else
		{
			samples[nextSampleIndex] = val;
			nextSampleIndex = (nextSampleIndex + 1) % MAX_HISTORY;
		}
	}

	inline int getSampleCount() const { return samples.size(); }

	inline float getSample(const int i) const { return samples[(nextSampleIndex + i) % MAX_HISTORY]; }

	float getSampleMin() const;
	float getSampleMax() const;
	float getAverage() const;
};

struct GraphParams
{
	void setRect(int ix, int iy, int iw, int ih, int ipad);
	void setValueRange(float minValue, float maxValue, int numDivisions, const std::string& units);

	int x;
	int y;
	int width;
	int height;
	int padding;

	float rangeMin;
	float rangeMax;
	int rangeDivisions;

	std::string units;
};

void drawGraphBackground(const GraphParams* p);
void drawGraph(const GraphParams* params, const ValueHistory* graph, int index, const char* label, const unsigned int color);
