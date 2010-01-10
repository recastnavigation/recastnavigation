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

#ifndef OFFMESHLINKTOOL_H
#define OFFMESHLINKTOOL_H

#include "Sample.h"

// Tool to create extra links for InputGeom

class OffMeshLinkTool : public SampleTool
{
	Sample* m_sample;
	float m_hitPos[3];
	bool m_hitPosSet;
	
public:
	OffMeshLinkTool();
	~OffMeshLinkTool();
	
	virtual int type() { return TOOL_OFFMESH_LINK; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* p, bool shift);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
};

#endif // OFFMESHLINKTOOL_H
