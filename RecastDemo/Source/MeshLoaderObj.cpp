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

#include "MeshLoaderObj.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#define _USE_MATH_DEFINES
#include <math.h>

rcMeshLoaderObj::rcMeshLoaderObj() :
	m_scale(1.0f)
{
}

rcMeshLoaderObj::~rcMeshLoaderObj()
{
}
		
void rcMeshLoaderObj::addVertex(float x, float y, float z, int& cap)
{
	m_vertexs.push_back({ x * m_scale, y * m_scale, z * m_scale });
}

void rcMeshLoaderObj::addTriangle(int a, int b, int c, int& cap)
{
	m_triangles.push_back({ a,b,c });
}

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\\':
				break;
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
				// else falls through
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}

static int parseFace(char* row, int* data, int n, int vcnt)
{
	int j = 0;
	while (*row != '\0')
	{
		// Skip initial white space
		while (*row != '\0' && (*row == ' ' || *row == '\t'))
			row++;
		char* s = row;
		// Find vertex delimiter and terminated the string there for conversion.
		while (*row != '\0' && *row != ' ' && *row != '\t')
		{
			if (*row == '/') *row = '\0';
			row++;
		}
		if (*s == '\0')
			continue;
		int vi = atoi(s);
		data[j++] = vi < 0 ? vi+vcnt : vi-1;
		if (j >= n) return j;
	}
	return j;
}

bool rcMeshLoaderObj::load(const std::string& filename)
{
	char* buf = 0;
	FILE* fp = fopen(filename.c_str(), "rb");
	if (!fp)
		return false;
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}
	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}
	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);

	if (readLen != 1)
	{
		delete[] buf;
		return false;
	}

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512];
	int face[32];
	float x,y,z;
	int nv;
	int vcap = 0;
	int tcap = 0;
	
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		// Skip comments
		if (row[0] == '#') continue;
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			sscanf(row+1, "%f %f %f", &x, &y, &z);
			addVertex(x, y, z, vcap);
		}
		if (row[0] == 'f')
		{
			// Faces
			nv = parseFace(row+1, face, 32, (int)m_vertexs.size());
			for (int i = 2; i < nv; ++i)
			{
				const int a = face[0];
				const int b = face[i-1];
				const int c = face[i];
				if (a < 0 || a >= (int)m_vertexs.size() || b < 0 || b >= (int)m_vertexs.size() || c < 0 || c >= (int)m_vertexs.size())
					continue;
				addTriangle(a, b, c, tcap);
			}
		}
	}

	delete [] buf;

	for (size_t i = 0; i < m_triangles.size(); ++i)
	{
		const auto& tri = m_triangles[i];
		const auto& v0 = m_vertexs[tri.v0];
		const auto& v1 = m_vertexs[tri.v1];
		const auto& v2 = m_vertexs[tri.v2];
		
		Vector3 e0, e1;
		e0.x = v1.x - v0.x;
		e0.y = v1.y - v0.y;
		e0.z = v1.z - v0.z;
		e1.x = v2.x - v0.x;
		e1.y = v2.y - v0.y;
		e1.z = v2.z - v0.z;

		Vector3 n;
		n.x = e0.y * e1.z - e0.z * e1.y;
		n.y = e0.z * e1.x - e0.x * e1.z;
		n.z = e0.x * e1.y - e0.y * e1.x;
		float d = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
		if (d > 0)
		{
			d = 1.0f / d;
			n.x *= d;
			n.y *= d;
			n.z *= d;
		}

		m_normals.push_back(n);
	}

	
	m_filename = filename;
	return true;
}
