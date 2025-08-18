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

#include "Sample.h"

/// Tool to create convex volumes for InputGeom
class ConvexVolumeTool : public SampleTool
{
	Sample* sample = nullptr;
	int areaType = SAMPLE_POLYAREA_GRASS;
	float polyOffset = 0.0f;
	float boxHeight = 6.0f;
	float boxDescent = 1.0f;

	static constexpr int MAX_PTS = 12;

	float points[MAX_PTS * 3] {};
	int numPoints = 0;
	int hull[MAX_PTS] {};
	int numHull = 0;

public:
	SampleToolType type() override { return SampleToolType::CONVEX_VOLUME; }
	void init(Sample* sample) override { this->sample = sample; }
	void reset() override
	{
		numPoints = 0;
		numHull = 0;
	}
	void drawMenuUI() override;
	void onClick(const float* s, const float* p, bool shift) override;
	void onToggle() override {}
	void singleStep() override {}
	void update(const float) override {}
	void render() override;
	void renderOverlay(double* proj, double* model, int* view) override;
};
