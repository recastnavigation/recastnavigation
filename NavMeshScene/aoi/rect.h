#ifndef __AOI_RECT_H__
#define __AOI_RECT_H__

#include "point.h"

namespace aoi
{
    enum EQuadrant
    {
        UnknowQuadrant = 0,
        RightTop = 1,    // Top right: quadrant 1
        LeftTop = 2,     // Top left: quadrant 2
        LeftDown = 3,    // Bottom left: Quadrant 3
        RightDown = 4,   // Bottom right: Quadrant 4
    };

    class Rect
    {
    public:

        Rect() : mLeft(0), mRight(0), mTop(0), mBottom(0), mMidX(0), mMidY(0) { }

        Rect(float left, float right, float bottom, float top)
            : mLeft(left)
            , mRight(right)
            , mTop(top)
            , mBottom(bottom)
            , mMidX(left + (right - left) / 2)
            , mMidY(bottom + (top - bottom) / 2)
        {
        }

        Rect(const Rect& other)
            : mLeft(other.mLeft)
            , mRight(other.mRight)
            , mTop(other.mTop)
            , mBottom(other.mBottom)
            , mMidX(other.mMidX)
            , mMidY(other.mMidY)
        {
        }

        Rect(Point center, Size halfExtents)
            : mLeft(center.X - halfExtents.X)
            , mRight(center.X + halfExtents.X)
            , mTop(center.Y + halfExtents.Y)
            , mBottom(center.Y - halfExtents.Y)
            , mMidX(mLeft + (mRight - mLeft) / 2)
            , mMidY(mBottom + (mTop - mBottom) / 2)
        {
        }

        inline void Reset()
        {
            mLeft = 0;
            mRight = 0;
            mTop = 0;
            mBottom = 0;
            mMidX = 0;
            mMidY = 0;
        }

        inline void Reset(float left, float right, float bottom, float top)
        {
            mLeft = left;
            mRight = right;
            mTop = top;
            mBottom = bottom;
            mMidX = left + (right - left) / 2;
            mMidY = bottom + (top - bottom) / 2;
        }

        inline float Left() const { return mLeft; }
        inline float Right() const { return mRight; }
        inline float Bottom() const { return mBottom; }
        inline float Top() const { return mTop; }
        inline float MidX() const { return mMidX; }
        inline float MidY() const { return mMidY; }

        inline bool Contains(const Rect& Rect) const
        {
            return (mLeft <= Rect.mLeft
                && mBottom <= Rect.mBottom
                && Rect.mRight <= mRight
                && Rect.mTop <= mTop);
        }

        inline bool Contains(float x, float y) const
        {
            return (x >= mLeft && x <= mRight
                && y >= mBottom && y <= mTop);
        }

        inline bool Contains(const Point& point) const
        {
            return Contains(point.X, point.Y);
        }

        inline bool Contains(const Point* point) const
        {
            return Contains(point->X, point->Y);
        }

        inline bool Intersects(const Rect& Rect) const
        {
            return !(mRight < Rect.mLeft
                || Rect.mRight < mLeft
                || mTop < Rect.mBottom
                || Rect.mTop < mBottom);
        }

        inline EQuadrant GetQuadrant(const Point& point)
        {
            return Contains(point) ? GetQuadrant2(point) : UnknowQuadrant;
        }

        inline EQuadrant GetQuadrant(const Point* point)
        {
            return GetQuadrant(*point);
        }

        inline EQuadrant GetQuadrant2(const Point& point)
        {
            return (point.Y >= mMidY
                ? (point.X >= mMidX ? RightTop : LeftTop)
                : (point.X >= mMidX ? RightDown : LeftDown));
        }

        inline EQuadrant GetQuadrant2(const Point* point)
        {
            return GetQuadrant2(*point);
        }

    private:
        float mLeft;
        float mRight;
        float mTop;
        float mBottom;
        float mMidX;
        float mMidY;
    };
}

#endif
