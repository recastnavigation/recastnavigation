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

/// Tool to create off-mesh connections for InputGeom
class OffMeshConnectionTool : public SampleTool
{
	Sample* sample = nullptr;
	float hitPos[3];
	bool hitPosSet = false;
	bool bidirectional = true;
	unsigned char oldFlags = 0;

public:
	OffMeshConnectionTool() = default;
	~OffMeshConnectionTool() override
	{
		if (sample)
		{
			sample->navMeshDrawFlags = oldFlags;
		}
	}

	SampleToolType type() override { return SampleToolType::OFFMESH_CONNECTION; }

	void init(Sample* sample) override;
	void reset() override;

	void singleStep() override {}
	void update(const float /*dt*/) override {}
	void render() override;

	void drawMenuUI() override;
	void drawOverlayUI(double* proj, double* model, int* view) override;

	void onClick(const float* rayStartTime, const float* rayHitPos, bool shift) override;
	void onToggle() override {}
};
