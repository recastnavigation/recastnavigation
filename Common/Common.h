#pragma once

#include <vector>

struct Vector2
{
	float x = 0;
	float y = 0;
};

struct Vector3
{
	float x = 0;
	float y = 0;
	float z = 0;

	void clear()
	{
		x = 0;
		y = 0;
		z = 0;
	}
};

struct Bounds
{
	Vector3 bmin;				///< The minimum bounds in world space. [(x, y, z)]
	Vector3 bmax;				///< The maximum bounds in world space. [(x, y, z)]
	float cs = 0;				///< The size of each cell. (On the xz-plane.)
	float ch = 0;				///< The height of each cell. (The minimum increment along the y-axis.)
};

// 一个三角面包含三个顶点索引
struct Triangle
{
	int v0 = 0;
	int v1 = 0;
	int v2 = 0;
};

