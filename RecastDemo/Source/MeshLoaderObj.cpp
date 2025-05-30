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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
char* readRow(char* buf, char* bufEnd, char* row, int len)
{
	// skip leading whitespace
	for (; buf < bufEnd; ++buf)
	{
		char c = *buf;
		if (c != '\\' && c != '\r' && c != '\n' && c != '\t' && c != ' ')
		{
			break;
		}
	}

	int n = 0;
	for (; buf < bufEnd; ++buf)
	{
		char c = *buf;

		if (c == '\n')
		{
			break;
		}

		if (c == '\\' || '\r')
		{
			// skip
			continue;
		}

		// Copy character
		row[n++] = c;
		if (n >= len - 1)
		{
			break;
		}
	}
	row[n] = '\0';
	return buf;
}

int readFace(char* row, int* data, int maxDataLen, int vertCount)
{
	int numVertices = 0;
	while (*row != '\0')
	{
		// Skip initial white space
		while (*row != '\0' && (*row == ' ' || *row == '\t'))
		{
			row++;
		}

		char* s = row;
		// Find vertex delimiter and terminate the string there for conversion.
		while (*row != '\0' && *row != ' ' && *row != '\t')
		{
			if (*row == '/')
			{
				*row = '\0';
			}
			row++;
		}

		if (*s == '\0')
		{
			continue;
		}

		int vertexIndex = atoi(s);
		data[numVertices++] = vertexIndex < 0 ? vertexIndex + vertCount : vertexIndex - 1;
		if (numVertices >= maxDataLen)
		{
			break;
		}
	}
	return numVertices;
}
}

bool MeshLoaderObj::load(const std::string& fileName)
{
	char* buf = 0;
	FILE* fp = fopen(filename.c_str(), "rb");
	if (!fp)
	{
		return false;
	}
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

	load(buf, bufSize);
	delete[] buf;
	filename = fileName;
	return true;
}

void MeshLoaderObj::load(char* buf, size_t bufLen)
{
	char* src = buf;
	char* srcEnd = buf + bufLen;
	char row[512];
	int face[32];
	float x, y, z;
	int numVertices;

	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = readRow(src, srcEnd, row, sizeof(row) / sizeof(row[0]));

		if (row[0] == '#')
		{
			// Comment
			continue;
		}
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			sscanf(row + 1, "%f %f %f", &x, &y, &z);
			verts.push_back(x);
			verts.push_back(y);
			verts.push_back(z);
		}
		if (row[0] == 'f')
		{
			// Face
			numVertices = readFace(row + 1, face, sizeof(face) / sizeof(face[0]), getVertCount());
			for (int i = 2; i < numVertices; ++i)
			{
				const int a = face[0];
				const int b = face[i - 1];
				const int c = face[i];
				if (a < 0 || a >= getVertCount() || b < 0 || b >= getVertCount() || c < 0 || c >= getVertCount())
				{
					continue;
				}

				tris.push_back(a);
				tris.push_back(b);
				tris.push_back(c);
			}
		}
	}

	// Calculate normals.
	normals.resize(getTriCount() * 3);
	for (int i = 0; i < tris.size(); i += 3)
	{
		const float* vertex0 = &verts[tris[i + 0] * 3];
		const float* vertex1 = &verts[tris[i + 1] * 3];
		const float* vertex2 = &verts[tris[i + 2] * 3];

		// Construct two triangle edges
		float edge0[3];
		float edge1[3];
		for (int j = 0; j < 3; ++j)
		{
			edge0[j] = vertex1[j] - vertex0[j];
			edge1[j] = vertex2[j] - vertex0[j];
		}

		float* normal = &normals[i];

		// Cross product
		normal[0] = edge0[1] * edge1[2] - edge0[2] * edge1[1];
		normal[1] = edge0[2] * edge1[0] - edge0[0] * edge1[2];
		normal[2] = edge0[0] * edge1[1] - edge0[1] * edge1[0];

		// Normalize
		float normalLength = sqrtf(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
		if (normalLength > 0)
		{
			normalLength = 1.0f / normalLength;
			normal[0] *= normalLength;
			normal[1] *= normalLength;
			normal[2] *= normalLength;
		}
	}
}
