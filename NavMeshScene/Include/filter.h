#ifndef __NMS_FILTER_H__
#define __NMS_FILTER_H__

#include <memory>

class dtQueryFilter;

namespace NavMeshScene {

    enum PolyAreas
    {
        POLYAREA_GROUND = 0,
        POLYAREA_WATER = 1,
        POLYAREA_ROAD = 2,
        POLYAREA_DOOR = 3,
        POLYAREA_GRASS = 4,
        POLYAREA_JUMP = 5,
    };

    enum PolyFlags
    {
        POLYFLAGS_WALK = 0x01,      // Ability to walk (ground, grass, road)
        POLYFLAGS_SWIM = 0x02,      // Ability to swim (water).
        POLYFLAGS_DOOR = 0x04,      // Ability to move through doors.
        POLYFLAGS_JUMP = 0x08,      // Ability to jump.
        POLYFLAGS_DISABLED = 0x10,  // Disabled polygon
        POLYFLAGS_ALL = 0xffff      // All abilities.
    };

    const float DEFAULT_AREA_COST_GROUND = 1.0f;
    const float DEFAULT_AREA_COST_WATER = 10.0f;
    const float DEFAULT_AREA_COST_ROAD = 1.0f;
    const float DEFAULT_AREA_COST_DOOR = 1.0f;
    const float DEFAULT_AREA_COST_GRASS = 2.0f;
    const float DEFAULT_AREA_COST_JUMP = 1.5f;

    const unsigned short DEFAULT_INCLUDE_FLAGS = POLYFLAGS_ALL ^ POLYFLAGS_DISABLED;
    const unsigned short DEFAULT_EXCLUDE_FLAGS = 0;


    class Filter {
    public:
        Filter();
        virtual ~Filter();

        dtQueryFilter& Get() { return *mFilter; }
        void SetAreaCost(const int i, const float cost);
        void SetIncludeFlags(const unsigned short flags);
        void SetExcludeFlags(const unsigned short flags);

    protected:
        std::unique_ptr<dtQueryFilter> mFilter;
    };

}

#endif