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
	bool hitPosSet = 0;
	bool bidir = true;
	unsigned char oldFlags = 0;

public:
	OffMeshConnectionTool() = default;
	~OffMeshConnectionTool()
	{
		if (sample)
		{
			sample->setNavMeshDrawFlags(oldFlags);
		}
	}

	SampleToolType type() override { return SampleToolType::OFFMESH_CONNECTION; }
	void init(Sample* sample) override;
	void reset() override;
	void handleMenu() override;
	void handleClick(const float* rayStartTime, const float* rayHitPos, bool shift) override;
	void handleToggle() override;
	void handleStep() override;
	void handleUpdate(const float dt) override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
};
