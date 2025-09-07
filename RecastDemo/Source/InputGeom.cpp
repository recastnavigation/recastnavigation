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

#include "InputGeom.h"

#include "PartitionedMesh.h"
#include "Recast.h"
#include "SampleInterfaces.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
bool intersectSegmentTriangle(const float* sp, const float* sq, const float* a, const float* b, const float* c, float& t)
{
	float ab[3];
	rcVsub(ab, b, a);
	float ac[3];
	rcVsub(ac, c, a);
	float qp[3];
	rcVsub(qp, sp, sq);

	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	float norm[3];
	rcVcross(norm, ab, ac);

	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = rcVdot(qp, norm);
	if (d <= 0.0f)
	{
		return false;
	}

	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	float ap[3];
	rcVsub(ap, sp, a);
	t = rcVdot(ap, norm);
	if (t < 0.0f)
	{
		return false;
	}
	if (t > d)
	{
		return false;
	}  // For segment; exclude this code line for a ray test

	// Compute barycentric coordinate components and test if within bounds
	float e[3];
	rcVcross(e, qp, ap);
	float v = rcVdot(ac, e);
	if (v < 0.0f || v > d)
	{
		return false;
	}
	float w = -rcVdot(ab, e);
	if (w < 0.0f || v + w > d)
	{
		return false;
	}

	// Segment/ray intersects triangle. Perform delayed division
	t /= d;

	return true;
}

bool isectSegAABB(const float* sp, const float* sq, const float* amin, const float* amax, float& tmin, float& tmax)
{
	static constexpr float EPS = 1e-6f;

	float d[3];
	rcVsub(d, sq, sp);
	tmin = 0.0;
	tmax = 1.0f;

	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			if (sp[i] < amin[i] || sp[i] > amax[i])
			{
				return false;
			}
		}
		else
		{
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			if (t1 > t2)
			{
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			tmin = std::max(t1, tmin);
			tmax = std::min(t2, tmax);
			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	return true;
}

char* parseRow(char* buf, char* bufEnd, char* row, int len)
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
		case '\n':
			if (start)
			{
				break;
			}
			done = true;
			break;
		case '\r':
			break;
		case '\t':
		case ' ':
			if (start)
			{
				break;
			}
			// else falls through
		default:
			start = false;
			row[n++] = c;
			if (n >= len - 1)
			{
				done = true;
			}
			break;
		}
	}
	row[n] = '\0';
	return buf;
}

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

		if (c == '\\' || c == '\r')
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

void Mesh::readFromObj(char* buf, size_t bufLen)
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
			const int vertCount = static_cast<int>(verts.size()) / 3;
			numVertices = readFace(row + 1, face, sizeof(face) / sizeof(face[0]), vertCount);
			for (int i = 2; i < numVertices; ++i)
			{
				const int a = face[0];
				const int b = face[i - 1];
				const int c = face[i];
				if (a < 0 || a >= vertCount || b < 0 || b >= vertCount || c < 0 || c >= vertCount)
				{
					continue;
				}

				tris.push_back(a);
				tris.push_back(b);
				tris.push_back(c);
			}
		}
	}

	// Calculate face normals.
	normals.resize(tris.size());
	for (int i = 0; i < static_cast<int>(tris.size()); i += 3)
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

bool InputGeom::loadMesh(rcContext* ctx, const std::string& filepath)
{
	FileIO file;
	if (!file.openForRead(filepath.c_str()))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath.c_str());
		return false;
	}

	size_t bufferLen = file.getFileSize();
	char* buffer = new char[bufferLen];

	if (!file.read(buffer, bufferLen))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath.c_str());
		return false;
	}

	filename = filepath;
	clearOffMeshConnections();

	convexVolumes.clear();

	mesh.reset();
	mesh.readFromObj(buffer, bufferLen);
	rcCalcBounds(mesh.verts.data(), mesh.getVertCount(), meshBoundsMin, meshBoundsMax);

	partitionedMesh = {}; // Reset the partitioned mesh
	partitionedMesh.PartitionMesh(mesh.verts.data(), mesh.tris.data(), mesh.getTriCount(), 256);
	return true;
}

bool InputGeom::loadGeomSet(rcContext* ctx, const std::string& filepath)
{
	FileIO file;
	if (!file.openForRead(filepath.c_str()))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath.c_str());
		return false;
	}

	size_t bufferLen = file.getFileSize();
	char* buffer = new char[bufferLen];

	if (!file.read(buffer, bufferLen))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath.c_str());
		return false;
	}

	bool result = loadGeomSet(ctx, buffer, bufferLen);

	delete[] buffer;
	return result;
}

bool InputGeom::loadGeomSet(rcContext* ctx, char* buffer, size_t bufferLen)
{
	clearOffMeshConnections();
	convexVolumes.clear();

	char* src = buffer;
	char* srcEnd = buffer + bufferLen;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));

		if (row[0] == 'f')
		{
			// File name.
			const char* name = row + 1;
			// Skip white spaces
			for (; *name && isspace(*name); ++name)
			{
			}
			if (*name)
			{
				if (!loadMesh(ctx, name))
				{
					return false;
				}
			}
		}
		else if (row[0] == 'c')
		{
			// Off-mesh connection
			float startPos[3];
			float endPos[3];
			int bidir;
			int area;
			int flags;
			float rad;
			sscanf(
				row + 1,
				"%f %f %f  %f %f %f %f %d %d %d",
				&startPos[0],
				&startPos[1],
				&startPos[2],
				&endPos[0],
				&endPos[1],
				&endPos[2],
				&rad,
				&bidir,
				&area,
				&flags);
			addOffMeshConnection(
				startPos,
				endPos,
				rad,
				static_cast<unsigned char>(bidir),
				static_cast<unsigned char>(area),
				static_cast<unsigned short>(flags));
		}
		else if (row[0] == 'v')
		{
			// Convex volumes
			ConvexVolume& vol = convexVolumes.emplace_back();
			sscanf(row + 1, "%d %d %f %f", &vol.nverts, &vol.area, &vol.hmin, &vol.hmax);
			for (int i = 0; i < vol.nverts; ++i)
			{
				row[0] = '\0';
				src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
				sscanf(row, "%f %f %f", &vol.verts[i * 3 + 0], &vol.verts[i * 3 + 1], &vol.verts[i * 3 + 2]);
			}
		}
		else if (row[0] == 's')
		{
			// Settings
			hasBuildSettings = true;
			sscanf(
				row + 1,
				"%f %f %f %f %f %f %f %f %f %f %d %f %f %d %f %f %f %f %f %f %f",
				&buildSettings.cellSize,
				&buildSettings.cellHeight,
				&buildSettings.agentHeight,
				&buildSettings.agentRadius,
				&buildSettings.agentMaxClimb,
				&buildSettings.agentMaxSlope,
				&buildSettings.regionMinSize,
				&buildSettings.regionMergeSize,
				&buildSettings.edgeMaxLen,
				&buildSettings.edgeMaxError,
				&buildSettings.vertsPerPoly,
				&buildSettings.detailSampleDist,
				&buildSettings.detailSampleMaxError,
				&buildSettings.partitionType,
				&buildSettings.navMeshBMin[0],
				&buildSettings.navMeshBMin[1],
				&buildSettings.navMeshBMin[2],
				&buildSettings.navMeshBMax[0],
				&buildSettings.navMeshBMax[1],
				&buildSettings.navMeshBMax[2],
				&buildSettings.tileSize);
		}
	}
	return true;
}

bool InputGeom::load(rcContext* ctx, const std::string& filepath)
{
	size_t extensionPos = filepath.find_last_of('.');
	if (extensionPos == std::string::npos)
	{
		return false;
	}

	std::string extension = filepath.substr(extensionPos);
	std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

	if (extension == ".gset")
	{
		return loadGeomSet(ctx, filepath);
	}
	if (extension == ".obj")
	{
		return loadMesh(ctx, filepath);
	}

	return false;
}

bool InputGeom::saveGeomSet(const BuildSettings* settings)
{
	if (mesh.verts.empty())
	{
		return false;
	}

	// Change extension
	std::string filepath = filename;
	size_t extPos = filepath.find_last_of('.');
	if (extPos != std::string::npos)
	{
		filepath = filepath.substr(0, extPos);
	}
	filepath += ".gset";

	FILE* fp = fopen(filepath.c_str(), "w");
	if (!fp)
	{
		return false;
	}

	// Store mesh filename.
	fprintf(fp, "f %s\n", filename.c_str());

	// Store settings if any
	if (settings)
	{
		fprintf(
			fp,
			"s %f %f %f %f %f %f %f %f %f %f %d %f %f %d %f %f %f %f %f %f %f\n",
			settings->cellSize,
			settings->cellHeight,
			settings->agentHeight,
			settings->agentRadius,
			settings->agentMaxClimb,
			settings->agentMaxSlope,
			settings->regionMinSize,
			settings->regionMergeSize,
			settings->edgeMaxLen,
			settings->edgeMaxError,
			settings->vertsPerPoly,
			settings->detailSampleDist,
			settings->detailSampleMaxError,
			settings->partitionType,
			settings->navMeshBMin[0],
			settings->navMeshBMin[1],
			settings->navMeshBMin[2],
			settings->navMeshBMax[0],
			settings->navMeshBMax[1],
			settings->navMeshBMax[2],
			settings->tileSize);
	}

	// Store off-mesh links.
	int offMeshConCount = static_cast<int>(offmeshConnId.size());
	for (int i = 0; i < offMeshConCount; ++i)
	{
		const float* v = &offmeshConnVerts[i * 3 * 2];
		const float rad = offmeshConnRadius[i];
		const int bidir = offmeshConnBidirectional[i];
		const int area = offmeshConnArea[i];
		const int flags = offmeshConnFlags[i];
		fprintf(fp, "c %f %f %f  %f %f %f  %f %d %d %d\n", v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir, area, flags);
	}

	// Convex volumes
	for (ConvexVolume& vol : convexVolumes)
	{
		fprintf(fp, "v %d %d %f %f\n", vol.nverts, vol.area, vol.hmin, vol.hmax);
		for (int j = 0; j < vol.nverts; ++j)
		{
			fprintf(fp, "%f %f %f\n", vol.verts[j * 3 + 0], vol.verts[j * 3 + 1], vol.verts[j * 3 + 2]);
		}
	}

	fclose(fp);

	return true;
}

bool InputGeom::raycastMesh(float* src, float* dst, float& tmin) const
{
	// Prune hit ray.
	float btmin;
	float btmax;
	if (!isectSegAABB(src, dst, meshBoundsMin, meshBoundsMax, btmin, btmax))
	{
		return false;
	}

	float p[]{p[0] = src[0] + (dst[0] - src[0]) * btmin, p[1] = src[2] + (dst[2] - src[2]) * btmin};
	float q[]{src[0] + (dst[0] - src[0]) * btmax, src[2] + (dst[2] - src[2]) * btmax};

	std::vector<int> overlappingNodes;
	partitionedMesh.GetNodesOverlappingSegment(p, q, overlappingNodes);
	if (overlappingNodes.empty())
	{
		return false;
	}

	tmin = 1.0f;
	bool hit = false;

	for (int nodeIndex : overlappingNodes)
	{
		const PartitionedMesh::Node& node = partitionedMesh.nodes[nodeIndex];
		const int* tris = &partitionedMesh.tris[node.triIndex * 3];
		const int ntris = node.numTris;

		for (int j = 0; j < ntris * 3; j += 3)
		{
			float t = 1;
			if (intersectSegmentTriangle(
					src,
					dst,
					&mesh.verts[tris[j] * 3],
					&mesh.verts[tris[j + 1] * 3],
					&mesh.verts[tris[j + 2] * 3],
					t))
			{
				tmin = std::min(t, tmin);
				hit = true;
			}
		}
	}

	return hit;
}

void InputGeom::addOffMeshConnection(
	const float* startPos,
	const float* endPos,
	const float radius,
	unsigned char bidirectional,
	unsigned char area,
	unsigned short flags)
{
	offmeshConnVerts.resize(offmeshConnVerts.size() + 3 * 2);
	float* v = &offmeshConnVerts[offmeshConnVerts.size() - 3 * 2];
	rcVcopy(&v[0], startPos);
	rcVcopy(&v[3], endPos);
	offmeshConnRadius.emplace_back(radius);
	offmeshConnBidirectional.emplace_back(bidirectional);
	offmeshConnArea.emplace_back(area);
	offmeshConnFlags.emplace_back(flags);
	offmeshConnId.emplace_back(1000 + (static_cast<unsigned int>(offmeshConnArea.size()) - 1));
}

void InputGeom::deleteOffMeshConnection(int i)
{
	offmeshConnVerts.erase(offmeshConnVerts.begin() + 3 * 2 * i);
	offmeshConnRadius.erase(offmeshConnRadius.begin() + i);
	offmeshConnBidirectional.erase(offmeshConnBidirectional.begin() + i);
	offmeshConnArea.erase(offmeshConnArea.begin() + i);
	offmeshConnFlags.erase(offmeshConnFlags.begin() + i);
	offmeshConnId.erase(offmeshConnId.begin() + i);
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, bool highlight)
{
	unsigned int conColor = duRGBA(192, 0, 128, 192);
	unsigned int baseColor = duRGBA(0, 0, 0, 64);
	dd->depthMask(false);

	dd->begin(DU_DRAW_LINES, 2.0f);
	int offMeshConCount = static_cast<int>(offmeshConnId.size());
	for (int i = 0; i < offMeshConCount; ++i)
	{
		float* v = &offmeshConnVerts[i * 3 * 2];

		dd->vertex(v[0], v[1], v[2], baseColor);
		dd->vertex(v[0], v[1] + 0.2f, v[2], baseColor);

		dd->vertex(v[3], v[4], v[5], baseColor);
		dd->vertex(v[3], v[4] + 0.2f, v[5], baseColor);

		duAppendCircle(dd, v[0], v[1] + 0.1f, v[2], offmeshConnRadius[i], baseColor);
		duAppendCircle(dd, v[3], v[4] + 0.1f, v[5], offmeshConnRadius[i], baseColor);

		if (highlight)
		{
			duAppendArc(
				dd,
				v[0],
				v[1],
				v[2],
				v[3],
				v[4],
				v[5],
				0.25f,
				(offmeshConnBidirectional[i] & 1) ? 0.6f : 0.0f,
				0.6f,
				conColor);
		}
	}
	dd->end();
	dd->depthMask(true);
}

void InputGeom::addConvexVolume(const float* verts, const int nverts, const float minh, const float maxh, unsigned char area)
{
	ConvexVolume vol;
	memcpy(vol.verts, verts, sizeof(float) * 3 * nverts);
	vol.hmin = minh;
	vol.hmax = maxh;
	vol.nverts = nverts;
	vol.area = area;
	convexVolumes.emplace_back(std::move(vol));
}

void InputGeom::deleteConvexVolume(int i)
{
	convexVolumes.erase(convexVolumes.begin() + i);
}

void InputGeom::drawConvexVolumes(struct duDebugDraw* dd)
{
	dd->depthMask(false);
	dd->begin(DU_DRAW_TRIS);

	for (const ConvexVolume& vol : convexVolumes)
	{
		unsigned int col = duTransCol(dd->areaToCol(vol.area), 32);
		for (int j = 0, k = vol.nverts - 1; j < vol.nverts; k = j++)
		{
			const float* va = &vol.verts[k * 3];
			const float* vb = &vol.verts[j * 3];

			dd->vertex(vol.verts[0], vol.hmax, vol.verts[2], col);
			dd->vertex(vb[0], vol.hmax, vb[2], col);
			dd->vertex(va[0], vol.hmax, va[2], col);

			dd->vertex(va[0], vol.hmin, va[2], duDarkenCol(col));
			dd->vertex(va[0], vol.hmax, va[2], col);
			dd->vertex(vb[0], vol.hmax, vb[2], col);

			dd->vertex(va[0], vol.hmin, va[2], duDarkenCol(col));
			dd->vertex(vb[0], vol.hmax, vb[2], col);
			dd->vertex(vb[0], vol.hmin, vb[2], duDarkenCol(col));
		}
	}

	dd->end();

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (const ConvexVolume& vol : convexVolumes)
	{
		unsigned int col = duTransCol(dd->areaToCol(vol.area), 220);
		for (int j = 0, k = vol.nverts - 1; j < vol.nverts; k = j++)
		{
			const float* va = &vol.verts[k * 3];
			const float* vb = &vol.verts[j * 3];
			dd->vertex(va[0], vol.hmin, va[2], duDarkenCol(col));
			dd->vertex(vb[0], vol.hmin, vb[2], duDarkenCol(col));
			dd->vertex(va[0], vol.hmax, va[2], col);
			dd->vertex(vb[0], vol.hmax, vb[2], col);
			dd->vertex(va[0], vol.hmin, va[2], duDarkenCol(col));
			dd->vertex(va[0], vol.hmax, va[2], col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (const ConvexVolume& vol : convexVolumes)
	{
		unsigned int col = duDarkenCol(duTransCol(dd->areaToCol(vol.area), 220));
		for (int j = 0; j < vol.nverts; ++j)
		{
			dd->vertex(vol.verts[j * 3 + 0], vol.verts[j * 3 + 1] + 0.1f, vol.verts[j * 3 + 2], col);
			dd->vertex(vol.verts[j * 3 + 0], vol.hmin, vol.verts[j * 3 + 2], col);
			dd->vertex(vol.verts[j * 3 + 0], vol.hmax, vol.verts[j * 3 + 2], col);
		}
	}
	dd->end();
	dd->depthMask(true);
}
