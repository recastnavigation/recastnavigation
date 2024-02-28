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

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <new>
#include <ranges>
#include <sstream>

#include "Recast.h"
#include "InputGeom.h"

#include "ChunkyTriMesh.h"
#include "DebugDraw.h"
#include "MeshLoaderObj.h"

static bool intersectSegmentTriangle(const float* sp, const float* sq,
                                     const float* a, const float* b, const float* c,
                                     float& t)
{
    float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
    rcVsub(ab, b, a);
    rcVsub(ac, c, a);
    rcVsub(qp, sp, sq);

    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    rcVcross(norm, ab, ac);

    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    const float d = rcVdot(qp, norm);
    if (d <= 0.0f) return false;

    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle
    rcVsub(ap, sp, a);
    t = rcVdot(ap, norm);
    if (t < 0.0f) return false;
    if (t > d) return false; // For segment; exclude this code line for a ray test

    // Compute barycentric coordinate components and test if within bounds
    rcVcross(e, qp, ap);
    const float v = rcVdot(ac, e);
    if (v < 0.0f || v > d) return false;
    if (const float w = -rcVdot(ab, e); w < 0.0f || v + w > d) return false;

    // Segment/ray intersects triangle. Perform delayed division
    t /= d;

    return true;
}

static char* parseRow(char* buf, const char* bufEnd, char* row, const int len)
{
    bool start = true;
    bool done = false;
    int n = 0;
    while (!done && buf < bufEnd)
    {
        const char c = *buf;
        buf++;
        // multirow
        switch (c)
        {
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
            if (n >= len - 1)
                done = true;
            break;
        }
    }
    row[n] = '\0';
    return buf;
}


InputGeom::InputGeom() :
    m_chunkyMesh(nullptr),
    m_mesh(nullptr), m_meshBMin{}, m_meshBMax{}, m_buildSettings(),
    m_hasBuildSettings(false), m_offMeshConVerts{}, m_offMeshConRads{}, m_offMeshConDirs{}, m_offMeshConAreas{},
    m_offMeshConFlags{},
    m_offMeshConId{},
    m_offMeshConCount(0), m_volumes{},
    m_volumeCount(0)
{
}

InputGeom::~InputGeom()
{
    delete m_chunkyMesh;
    delete m_mesh;
}

bool InputGeom::loadMesh(rcContext* ctx, const std::string& filePath)
{
    if (m_mesh)
    {
        delete m_chunkyMesh;
        m_chunkyMesh = nullptr;
        delete m_mesh;
        m_mesh = nullptr;
    }
    m_offMeshConCount = 0;
    m_volumeCount = 0;

    m_mesh = new (std::nothrow) rcMeshLoaderObj;
    if (!m_mesh)
    {
        ctx->log(RC_LOG_ERROR, "loadMesh: Out of memory 'm_mesh'.");
        return false;
    }
    if (!m_mesh->load(filePath))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filePath.c_str());
        return false;
    }

    rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), m_meshBMin, m_meshBMax);

    m_chunkyMesh = new (std::nothrow) rcChunkyTriMesh;
    if (!m_chunkyMesh)
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
        return false;
    }
    if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, m_chunkyMesh))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
        return false;
    }

    return true;
}

bool InputGeom::loadGeomSet(rcContext* ctx, const std::string& filePath)
{
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        return false;
    }

    const std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (fileSize < 0) {
        return false;
    }

    auto *const buf = new (std::nothrow) char[fileSize];
    if (!buf)
    {
        file.close();
        return false;
    }
    file.read(buf, fileSize);
    const std::size_t readLen = file.gcount();
    file.close();
    if (readLen != fileSize)
    {
        delete[] buf;
        return false;
    }

    m_offMeshConCount = 0;
    m_volumeCount = 0;
    delete m_mesh;
    m_mesh = nullptr;

    char* src = buf;
    const char* srcEnd = buf + fileSize;
    char row[512];
    while (src < srcEnd)
    {
        // Parse one row
        row[0] = '\0';
        src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
        char command = row[0];
        std::istringstream rowStream(row + 1);
        switch (command) {
            case 'f': {
                // File name
                std::string name;
                rowStream >> name;
                if (!name.empty())
                {
                    if (!loadMesh(ctx, name))
                    {
                        delete[] buf;
                        return false;
                    }
                }
                break;
            }
            case 'c': {
                // Off-mesh connection
                if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
                {
                    float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
                    int bidir, area = 0, flags = 0;
                    float rad;
                    rowStream >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5] >> rad >> bidir >> area >> flags;
                    m_offMeshConRads[m_offMeshConCount] = rad;
                    m_offMeshConDirs[m_offMeshConCount] = static_cast<unsigned char>(bidir);
                    m_offMeshConAreas[m_offMeshConCount] = static_cast<unsigned char>(area);
                    m_offMeshConFlags[m_offMeshConCount] = static_cast<uint16_t>(flags);
                    m_offMeshConCount++;
                }
                break;
            }
            case 'v': {
                // Convex volumes
                if (m_volumeCount < MAX_VOLUMES) {
                    auto&[verts, hmin, hmax, nverts, area] = m_volumes[m_volumeCount++];
                    rowStream >> nverts >> area >> hmin >> hmax;
                    for (int i = 0; i < nverts; ++i)
                    {
                        row[0] = '\0';
                        src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
                        std::istringstream vertStream(row);
                        vertStream >> verts[0] >>verts[1] >> verts[2];
                    }
                }
                break;
            }
            case 's': {
                m_hasBuildSettings = true;
                rowStream >> m_buildSettings.cellSize >> m_buildSettings.cellHeight
                          >> m_buildSettings.agentHeight >> m_buildSettings.agentRadius
                          >> m_buildSettings.agentMaxClimb >> m_buildSettings.agentMaxSlope
                          >> m_buildSettings.regionMinSize >> m_buildSettings.regionMergeSize
                          >> m_buildSettings.edgeMaxLen >> m_buildSettings.edgeMaxError
                          >> m_buildSettings.vertsPerPoly >> m_buildSettings.detailSampleDist
                          >> m_buildSettings.detailSampleMaxError >> m_buildSettings.partitionType
                          >> m_buildSettings.navMeshBMin[0] >> m_buildSettings.navMeshBMin[1] >> m_buildSettings.navMeshBMin[2]
                          >> m_buildSettings.navMeshBMax[0] >> m_buildSettings.navMeshBMax[1] >> m_buildSettings.navMeshBMax[2]
                          >> m_buildSettings.tileSize;
                break;
            }
            default: {
                break;
            }
        }
    }
    delete [] buf;

    return true;
}

bool InputGeom::load(rcContext* ctx, const std::string& filepath)
{
    const std::size_t extensionPos = filepath.find_last_of('.');
    if (extensionPos == std::string::npos)
        return false;

    std::string extension = filepath.substr(extensionPos);
    std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

    if (extension == ".gset")
        return loadGeomSet(ctx, filepath);
    if (extension == ".obj")
        return loadMesh(ctx, filepath);

    return false;
}

bool InputGeom::saveGeomSet(const BuildSettings* settings) const
{
    if (!m_mesh) return false;

    // Change extension
    std::string filepath = m_mesh->getFileName();
    if (const std::size_t extPos = filepath.find_last_of('.'); extPos != std::string::npos)
        filepath = filepath.substr(0, extPos);

    filepath += ".gset";

    std::ofstream file{filepath};
    if (!file.is_open()) return false;

    // Store mesh filename.
    file << "f " << m_mesh->getFileName() << "\n";

    // Store settings if any
    if (settings) {
        file << "s "
             << settings->cellSize << " "
             << settings->cellHeight << " "
             << settings->agentHeight << " "
             << settings->agentRadius << " "
             << settings->agentMaxClimb << " "
             << settings->agentMaxSlope << " "
             << settings->regionMinSize << " "
             << settings->regionMergeSize << " "
             << settings->edgeMaxLen << " "
             << settings->edgeMaxError << " "
             << settings->vertsPerPoly << " "
             << settings->detailSampleDist << " "
             << settings->detailSampleMaxError << " "
             << settings->partitionType << " "
             << settings->navMeshBMin[0] << " "
             << settings->navMeshBMin[1] << " "
             << settings->navMeshBMin[2] << " "
             << settings->navMeshBMax[0] << " "
             << settings->navMeshBMax[1] << " "
             << settings->navMeshBMax[2] << " "
             << settings->tileSize << "\n";
    }

    // Store off-mesh links.
    for (int i = 0; i < m_offMeshConCount; ++i) {
        const float* v = &m_offMeshConVerts[i * 3 * 2];
        const float rad = m_offMeshConRads[i];
        const int bidir = m_offMeshConDirs[i];
        const int area = m_offMeshConAreas[i];
        const int flags = m_offMeshConFlags[i];
        file << "c " << v[0] << " " << v[1] << " " << v[2] << " "
             << v[3] << " " << v[4] << " " << v[5] << " "
             << rad << " " << bidir << " " << area << " " << flags << "\n";
    }

    // Convex volumes
    for (const auto&[verts, hmin, hmax, nverts, area] : m_volumes) {
        file << "v " << nverts << " " << area << " " << hmin << " " << hmax << "\n";
        for (int j = 0; j < nverts; ++j) {
            file << verts[j * 3 + 0] << " " << verts[j * 3 + 1] << " " << verts[j * 3 + 2] << "\n";
        }
    }

    file.close();


    return true;
}

static bool isectSegAABB(const float* sp, const float* sq,
                         const float* amin, const float* amax,
                         float& tmin, float& tmax)
{
    static constexpr float EPS = 1e-6f;

    float d[3];
    d[0] = sq[0] - sp[0];
    d[1] = sq[1] - sp[1];
    d[2] = sq[2] - sp[2];
    tmin = 0.0;
    tmax = 1.0f;

    for (int i = 0; i < 3; i++)
    {
        if (std::abs(d[i]) < EPS)
        {
            if (sp[i] < amin[i] || sp[i] > amax[i])
                return false;
        }
        else
        {
            const float ood = 1.0f / d[i];
            float t1 = (amin[i] - sp[i]) * ood;
            float t2 = (amax[i] - sp[i]) * ood;
            if (t1 > t2)
            {
                const float tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }

    return true;
}


bool InputGeom::raycastMesh(const float* src, const float* dst, float& tmin) const
{
    // Prune hit ray.
    float btmin, btmax;
    if (!isectSegAABB(src, dst, m_meshBMin, m_meshBMax, btmin, btmax))
        return false;
    float p[2], q[2];
    p[0] = src[0] + (dst[0] - src[0]) * btmin;
    p[1] = src[2] + (dst[2] - src[2]) * btmin;
    q[0] = src[0] + (dst[0] - src[0]) * btmax;
    q[1] = src[2] + (dst[2] - src[2]) * btmax;

    int cid[512];
    const int ncid = rcGetChunksOverlappingSegment(m_chunkyMesh, p, q, cid, 512);
    if (!ncid)
        return false;

    tmin = 1.0f;
    bool hit = false;
    const float* verts = m_mesh->getVerts();

    for (int i = 0; i < ncid; ++i)
    {
        const auto& [bmin, bmax, index, n]  {m_chunkyMesh->nodes[cid[i]]};
        const int* tris = &m_chunkyMesh->tris[index * 3];
        const int ntris = n;

        for (int j = 0; j < ntris * 3; j += 3)
        {
            if (float t = 1; intersectSegmentTriangle(src, dst,
                                                      &verts[tris[j] * 3],
                                                      &verts[tris[j + 1] * 3],
                                                      &verts[tris[j + 2] * 3], t))
            {
                if (t < tmin)
                    tmin = t;
                hit = true;
            }
        }
    }

    return hit;
}

void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
                                     const unsigned char bidir, const unsigned char area, const uint16_t flags)
{
    if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
    float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
    m_offMeshConRads[m_offMeshConCount] = rad;
    m_offMeshConDirs[m_offMeshConCount] = bidir;
    m_offMeshConAreas[m_offMeshConCount] = area;
    m_offMeshConFlags[m_offMeshConCount] = flags;
    m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
    rcVcopy(&v[0], spos);
    rcVcopy(&v[3], epos);
    m_offMeshConCount++;
}

void InputGeom::deleteOffMeshConnection(const int i)
{
    m_offMeshConCount--;
    const float* src = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
    float* dst = &m_offMeshConVerts[i * 3 * 2];
    rcVcopy(&dst[0], &src[0]);
    rcVcopy(&dst[3], &src[3]);
    m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
    m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
    m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
    m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, const bool hilight) const
{
    const uint32_t conColor = duRGBA(192, 0, 128, 192);
    const uint32_t baseColor = duRGBA(0, 0, 0, 64);
    dd->depthMask(false);

    dd->begin(DU_DRAW_LINES, 2.0f);
    for (int i = 0; i < m_offMeshConCount; ++i)
    {
        const float* v = &m_offMeshConVerts[i * 3 * 2];

        dd->vertex(v[0], v[1], v[2], baseColor);
        dd->vertex(v[0], v[1] + 0.2f, v[2], baseColor);

        dd->vertex(v[3], v[4], v[5], baseColor);
        dd->vertex(v[3], v[4] + 0.2f, v[5], baseColor);

        duAppendCircle(dd, v[0], v[1] + 0.1f, v[2], m_offMeshConRads[i], baseColor);
        duAppendCircle(dd, v[3], v[4] + 0.1f, v[5], m_offMeshConRads[i], baseColor);

        if (hilight)
        {
            duAppendArc(dd, v[0], v[1], v[2], v[3], v[4], v[5], 0.25f,
                        (m_offMeshConDirs[i] & 1) ? 0.6f : 0.0f, 0.6f, conColor);
        }
    }
    dd->end();

    dd->depthMask(true);
}

void InputGeom::addConvexVolume(const float* verts, const int nverts,
                                const float minh, const float maxh, const unsigned char area)
{
    if (m_volumeCount >= MAX_VOLUMES) return;
    ConvexVolume* vol = &m_volumes[m_volumeCount++];
    std::memset(vol, 0, sizeof(ConvexVolume));
    std::memcpy(vol->verts, verts, sizeof(float) * 3 * nverts);
    vol->hmin = minh;
    vol->hmax = maxh;
    vol->nverts = nverts;
    vol->area = area;
}

void InputGeom::deleteConvexVolume(const int i)
{
    m_volumeCount--;
    m_volumes[i] = m_volumes[m_volumeCount];
}

void InputGeom::drawConvexVolumes(duDebugDraw* dd, bool /*hilight*/) const
{
    dd->depthMask(false);

    dd->begin(DU_DRAW_TRIS);

    for (int i = 0; i < m_volumeCount; ++i)
    {
        const ConvexVolume* vol = &m_volumes[i];
        const uint32_t col = duTransCol(dd->areaToCol(vol->area), 32);
        for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
        {
            const float* va = &vol->verts[k * 3];
            const float* vb = &vol->verts[j * 3];

            dd->vertex(vol->verts[0], vol->hmax, vol->verts[2], col);
            dd->vertex(vb[0], vol->hmax, vb[2], col);
            dd->vertex(va[0], vol->hmax, va[2], col);

            dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
            dd->vertex(va[0], vol->hmax, va[2], col);
            dd->vertex(vb[0], vol->hmax, vb[2], col);

            dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
            dd->vertex(vb[0], vol->hmax, vb[2], col);
            dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
        }
    }

    dd->end();

    dd->begin(DU_DRAW_LINES, 2.0f);
    for (int i = 0; i < m_volumeCount; ++i)
    {
        const ConvexVolume* vol = &m_volumes[i];
        const uint32_t col = duTransCol(dd->areaToCol(vol->area), 220);
        for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
        {
            const float* va = &vol->verts[k * 3];
            const float* vb = &vol->verts[j * 3];
            dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
            dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
            dd->vertex(va[0], vol->hmax, va[2], col);
            dd->vertex(vb[0], vol->hmax, vb[2], col);
            dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
            dd->vertex(va[0], vol->hmax, va[2], col);
        }
    }
    dd->end();

    dd->begin(DU_DRAW_POINTS, 3.0f);
    for (int i = 0; i < m_volumeCount; ++i)
    {
        const ConvexVolume* vol = &m_volumes[i];
        const uint32_t col = duDarkenCol(duTransCol(dd->areaToCol(vol->area), 220));
        for (int j = 0; j < vol->nverts; ++j)
        {
            dd->vertex(vol->verts[j * 3 + 0], vol->verts[j * 3 + 1] + 0.1f, vol->verts[j * 3 + 2], col);
            dd->vertex(vol->verts[j * 3 + 0], vol->hmin, vol->verts[j * 3 + 2], col);
            dd->vertex(vol->verts[j * 3 + 0], vol->hmax, vol->verts[j * 3 + 2], col);
        }
    }
    dd->end();


    dd->depthMask(true);
}
