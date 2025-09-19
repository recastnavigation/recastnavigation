#ifndef __AOI_POINT_H__
#define __AOI_POINT_H__

namespace aoi
{
    class Point
    {
    public:
        Point() : X(0), Y(0) {}
        Point(float x, float y) : X(x), Y(y) {}
        Point(const Point& other) : X(other.X), Y(other.Y) {}

        inline bool IsZero() const { return X == 0 && Y == 0; }

    public:
        float X;
        float Y;
    };

    typedef Point Size;
}

#endif