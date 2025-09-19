#include "filter.h"
#include <DetourNavMeshQuery.h>

namespace NavMeshScene {

    Filter::Filter()
        : mFilter(new dtQueryFilter())
    {
        mFilter->setAreaCost(POLYAREA_GROUND, DEFAULT_AREA_COST_GROUND);
        mFilter->setAreaCost(POLYAREA_WATER, DEFAULT_AREA_COST_WATER);
        mFilter->setAreaCost(POLYAREA_ROAD, DEFAULT_AREA_COST_ROAD);
        mFilter->setAreaCost(POLYAREA_DOOR, DEFAULT_AREA_COST_DOOR);
        mFilter->setAreaCost(POLYAREA_GRASS, DEFAULT_AREA_COST_GRASS);
        mFilter->setAreaCost(POLYAREA_JUMP, DEFAULT_AREA_COST_JUMP);

        mFilter->setIncludeFlags(DEFAULT_INCLUDE_FLAGS);
        mFilter->setExcludeFlags(DEFAULT_EXCLUDE_FLAGS);
    }

    Filter::~Filter() {

    }

    void Filter::SetAreaCost(const int i, const float cost) {
        mFilter->setAreaCost(i, cost);
    }

    void Filter::SetIncludeFlags(const unsigned short flags) {
        mFilter->setIncludeFlags(flags);
    }

    void Filter::SetExcludeFlags(const unsigned short flags) {
        mFilter->setExcludeFlags(flags);
    }

}