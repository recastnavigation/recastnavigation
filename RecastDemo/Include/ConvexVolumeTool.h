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

// Tool to create convex volumess for InputGeom

class ConvexVolumeTool final : public SampleTool
{
	Sample* m_sample{};
	int m_areaType{SAMPLE_POLYAREA_GRASS};
	float m_polyOffset{};
	float m_boxHeight{0.6f};
	float m_boxDescent{0.1f};
	
	static constexpr int MAX_PTS = 12;
	float m_pts[MAX_PTS*3]{};
	int m_npts{};
	int m_hull[MAX_PTS]{};
	int m_nhull{};
	
public:
	ConvexVolumeTool()=default;

  int type() override { return TOOL_CONVEX_VOLUME; }
  void init(Sample *sample) override;
  void reset() override;
  void handleMenu() override;
  void handleClick(const float *s, const float *p, bool shift) override;
  void handleToggle() override;
  void handleStep() override;
  void handleUpdate(float dt) override;
  void handleRender() override;
  void handleRenderOverlay(double *proj, double *model, int *view) override;
};
