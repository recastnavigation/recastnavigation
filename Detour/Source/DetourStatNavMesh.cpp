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

#include "DetourStatNavMesh.h"
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////////////

template<class T> inline void swap(T& a, T& b) { T t = a; a = b; b = t; }
template<class T> inline T min(T a, T b) { return a < b ? a : b; }
template<class T> inline T max(T a, T b) { return a > b ? a : b; }
template<class T> inline T abs(T a) { return a < 0 ? -a : a; }
template<class T> inline T sqr(T a) { return a*a; }
template<class T> inline T clamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

// Some vector utils
inline void vcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]; 
}

inline float vdot(const float* v1, const float* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

inline void vsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]-v2[0];
	dest[1] = v1[1]-v2[1];
	dest[2] = v1[2]-v2[2];
}

inline void vmin(float* mn, const float* v)
{
	mn[0] = min(mn[0], v[0]);
	mn[1] = min(mn[1], v[1]);
	mn[2] = min(mn[2], v[2]);
}

inline void vmax(float* mx, const float* v)
{
	mx[0] = max(mx[0], v[0]);
	mx[1] = max(mx[1], v[1]);
	mx[2] = max(mx[2], v[2]);
}

inline void vcopy(float* dest, const float* a)
{
	dest[0] = a[0];
	dest[1] = a[1];
	dest[2] = a[2];
}

inline float vdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx*dx + dy*dy + dz*dz;
}

inline void vnormalize(float* v)
{
	float d = 1.0f / sqrtf(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

inline bool vequal(const float* p0, const float* p1)
{
	static const float thr = sqr(1.0f/16384.0f);
	const float d = vdistSqr(p0, p1);
	return d < thr;
}

inline int nextPow2(int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline float vdot2D(const float* u, const float* v)
{
	return u[0]*v[0] + u[2]*v[2];
}
inline float vperp2D(const float* u, const float* v)
{
	return u[2]*v[0] - u[0]*v[2];
}

inline float triArea2D(const float* a, const float* b, const float* c)
{
	return ((b[0]*a[2] - a[0]*b[2]) + (c[0]*b[2] - b[0]*c[2]) + (a[0]*c[2] - c[0]*a[2])) * 0.5f;
}

static void closestPtPointTriangle(float* closest, const float* p,
								   const float* a, const float* b, const float* c)
{
	// Check if P in vertex region outside A
	float ab[3], ac[3], ap[3];
	vsub(ab, b, a);
	vsub(ac, c, a);
	vsub(ap, p, a);
	float d1 = vdot(ab, ap);
	float d2 = vdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		vcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	float bp[3];
	vsub(bp, p, b);
	float d3 = vdot(ab, bp);
	float d4 = vdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		vcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}
	
	// Check if P in vertex region outside C
	float cp[3];
	vsub(cp, p, c);
	float d5 = vdot(ab, cp);
	float d6 = vdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		vcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

static bool intersectSegmentPoly2D(const float* p0, const float* p1,
								   const float* verts, int nverts,
								   float& tmin, float& tmax,
								   int& segMin, int& segMax)
{
	static const float EPS = 0.00000001f;
	
	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	
	float dir[3];
	vsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		float edge[3], diff[3];
		vsub(edge, &verts[i*3], &verts[j*3]);
		vsub(diff, p0, &verts[j*3]);
		float n = vperp2D(edge, diff);
		float d = -vperp2D(edge, dir);
		if (fabs(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

static float distancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	
	return dx*dx + dz*dz;
}

static void calcPolyCenter(float* tc, const dtPoly* p, const float* verts)
{
	tc[0] = 0.0f;
	tc[1] = 0.0f;
	tc[2] = 0.0f;
	for (int j = 0; j < (int)p->nv; ++j)
	{
		const float* v = &verts[p->v[j]*3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const float s = 1.0f / p->nv;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}


//////////////////////////////////////////////////////////////////////////////////////////
struct dtNode
{
	enum dtNodeFlags
	{
		OPEN = 0x01,
		CLOSED = 0x02,
	};
	dtNode* parent;
	unsigned short cost;
	unsigned short total;
	unsigned short id;
	unsigned short flags;
};

class dtNodePool
{
public:
	dtNodePool(int maxNodes, int hashSize);
	~dtNodePool();
	inline void operator=(const dtNodePool&) {}
	void clear();
	dtNode* getNode(unsigned short id);
	const dtNode* findNode(unsigned short id) const;
	
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtNode)*m_maxNodes +
		sizeof(unsigned short)*m_maxNodes +
		sizeof(unsigned short)*m_hashSize;
	}
	
private:
	inline unsigned int hashint(unsigned int a) const
	{
		a += ~(a<<15);
		a ^=  (a>>10);
		a +=  (a<<3);
		a ^=  (a>>6);
		a += ~(a<<11);
		a ^=  (a>>16);
		return a;
	}
	
	dtNode* m_nodes;
	unsigned short* m_first;
	unsigned short* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

dtNodePool::dtNodePool(int maxNodes, int hashSize) :
	m_maxNodes(maxNodes),
	m_hashSize(hashSize),
	m_nodes(0),
	m_first(0),
	m_next(0)
{
	m_nodes = new dtNode[m_maxNodes];
	m_next = new unsigned short[m_maxNodes];
	m_first = new unsigned short[hashSize];
	memset(m_first, 0xff, sizeof(unsigned short)*m_hashSize);
	memset(m_next, 0xff, sizeof(unsigned short)*m_maxNodes);
}

dtNodePool::~dtNodePool()
{
	delete [] m_nodes;
	delete [] m_next;
	delete [] m_first;
}

void dtNodePool::clear()
{
	memset(m_first, 0xff, sizeof(unsigned short)*m_hashSize);
	m_nodeCount = 0;
}

const dtNode* dtNodePool::findNode(unsigned short id) const
{
	unsigned int bucket = hashint((unsigned int)id) & (m_hashSize-1);
	unsigned short i = m_first[bucket];
	while (i != 0xffff)
	{
		if (m_nodes[i].id == id)
			return &m_nodes[i];
		i = m_next[i];
	}
	return 0;
}

dtNode* dtNodePool::getNode(unsigned short id)
{
	unsigned int bucket = hashint((unsigned int)id) & (m_hashSize-1);
	unsigned short i = m_first[bucket];
	dtNode* node = 0;
	while (i != 0xffff)
	{
		if (m_nodes[i].id == id)
			return &m_nodes[i];
		i = m_next[i];
	}
	
	if (m_nodeCount >= m_maxNodes)
		return 0;
	
	i = (unsigned short)m_nodeCount;
	m_nodeCount++;
	
	// Init node
	node = &m_nodes[i];
	node->parent = 0;
	node->cost = 0;
	node->total = 0;
	node->id = id;
	node->flags = 0;
	
	m_next[i] = m_first[bucket];
	m_first[bucket] = i;
	
	return node;
}


//////////////////////////////////////////////////////////////////////////////////////////
class dtNodeQueue
{
public:
	dtNodeQueue(int n);
	~dtNodeQueue();
	inline void operator=(dtNodeQueue&) {}
	
	inline void clear()
	{
		m_size = 0;
	}
	
	inline dtNode* top()
	{
		return m_heap[0];
	}
	
	inline dtNode* pop()
	{
		dtNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	inline void push(dtNode* node)
	{
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	inline void modify(dtNode* node)
	{
		for (unsigned i = 0; i < m_size; ++i)
		{
			if (m_heap[i] == node)
			{
				bubbleUp(i, node);
				return;
			}
		}
	}
	
	inline bool empty() const { return m_size == 0; }
	
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtNode*)*(m_capacity+1);
	}
	
	
private:
	void bubbleUp(int i, dtNode* node);
	void trickleDown(int i, dtNode* node);
	
	dtNode** m_heap;
	const int m_capacity;
	int m_size;
};		

dtNodeQueue::dtNodeQueue(int n) :
	m_capacity(n),
	m_size(0),
	m_heap(0)
{
	m_heap = new dtNode*[m_capacity+1];
}

dtNodeQueue::~dtNodeQueue()
{
	delete [] m_heap;
}

void dtNodeQueue::bubbleUp(int i, dtNode* node)
{
	int parent = (i-1)/2;
	// note: (index > 0) means there is a parent
	while ((i > 0) && (m_heap[parent]->total > node->total))
	{
		m_heap[i] = m_heap[parent];
		i = parent;
		parent = (i-1)/2;
	}
	m_heap[i] = node;
}
		
void dtNodeQueue::trickleDown(int i, dtNode* node)
{
	int child = (i*2)+1;
	while (child < m_size)
	{
		if (((child+1) < m_size) && 
			(m_heap[child]->total > m_heap[child+1]->total))
		{
			child++;
		}
		m_heap[i] = m_heap[child];
		i = child;
		child = (i*2)+1;
	}
	bubbleUp(i, node);
}


//////////////////////////////////////////////////////////////////////////////////////////
dtStatNavMesh::dtStatNavMesh() :
	m_header(0),
	m_polys(0),
	m_verts(0),
	m_bvtree(0),
	m_nodePool(0),
	m_openList(0),
	m_data(0),
	m_dataSize(0)
{
}

dtStatNavMesh::~dtStatNavMesh()
{
	delete m_nodePool;
	delete m_openList;
	if (m_data)
		delete [] m_data;
}

bool dtStatNavMesh::init(unsigned char* data, int dataSize, bool ownsData)
{
	m_header = (dtStatNavMeshHeader*)data;
	if (m_header->magic != DT_NAVMESH_MAGIC)
	{
		return false;
	}
	if (m_header->version != DT_NAVMESH_VERSION)
	{
		return false;
	}

	const int headerSize = sizeof(dtStatNavMeshHeader);
	const int vertsSize = sizeof(float)*3*m_header->nverts;
	const int polysSize = sizeof(dtPoly)*m_header->npolys;
	
	m_verts = (float*)(data + headerSize);
	m_polys = (dtPoly*)(data + headerSize + vertsSize);
	m_bvtree = (dtBVNode*)(data + headerSize + vertsSize + polysSize);
	
	m_nodePool = new dtNodePool(2048, 256);
	if (!m_nodePool)
		return false;
		
	m_openList = new dtNodeQueue(2048);
	if (!m_openList)
		return false;
	
	if (ownsData)
	{
		m_data = data;
		m_dataSize = dataSize;
	}

	return true;
}

unsigned short dtStatNavMesh::getCost(dtPolyRef from, dtPolyRef to) const
{
	const dtPoly* fromPoly = getPoly(from-1);
	const dtPoly* toPoly = getPoly(to-1);
	float fromPc[3], toPc[3];
	calcPolyCenter(fromPc, fromPoly, m_verts);
	calcPolyCenter(toPc, toPoly, m_verts);
	int cost = (int)sqrtf(sqr(fromPc[0]-toPc[0]) + sqr(fromPc[2]-toPc[2]));
	if (cost < 1) cost = 1;
	if (cost > 0xffff) cost = 0xffff;
	return cost;
}

const dtPoly* dtStatNavMesh::getPolyByRef(dtPolyRef ref) const
{
	if (!m_header || ref == 0 || (int)ref > m_header->npolys) return 0;
	return &m_polys[ref-1];
}

int dtStatNavMesh::findPath(dtPolyRef startRef, dtPolyRef endRef,
							dtPolyRef* path, const int maxPathSize)
{
	if (!startRef || !endRef)
		return 0;

	if (!maxPathSize)
		return 0;

	if (startRef == endRef)
	{
		path[0] = startRef;
		return 1;
	}

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = getCost(startRef, endRef);
	startNode->id = startRef;
	startNode->flags = dtNode::OPEN;
	m_openList->push(startNode);

	dtNode* lastBestNode = startNode;
	unsigned short lastBestNodeCost = startNode->total;
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
	
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		const dtPoly* poly = getPoly(bestNode->id-1);
		for (int i = 0; i < (int)poly->nv; ++i)
		{
			dtPolyRef neighbour = poly->n[i];
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->parent && bestNode->parent->id == neighbour)
					continue;

				dtNode newNode;
				newNode.parent = bestNode;
				newNode.id = neighbour;
				newNode.cost = bestNode->cost + getCost(newNode.parent->id, newNode.id);
				unsigned short costToGoal = getCost(newNode.id, endRef);
				newNode.total = newNode.cost + costToGoal;

				dtNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;
						
				if (!((actualNode->flags & dtNode::OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & dtNode::CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= ~dtNode::CLOSED;
					actualNode->parent = newNode.parent;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;

					if (costToGoal < lastBestNodeCost)
					{
						lastBestNodeCost = costToGoal;
						lastBestNode = actualNode;
					}

					if (actualNode->flags & dtNode::OPEN)
					{
						m_openList->modify(actualNode);
					}
					else
					{
						actualNode->flags = dtNode::OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
	}

	// Reverse the path.
	dtNode* prev = 0;
	dtNode* node = lastBestNode;
	do
	{
		dtNode* next = node->parent;
		node->parent = prev;
		prev = node;
		node = next;
	}
	while (node);
	
	// Store path
	node = prev;
	int n = 0;
	do
	{
		path[n++] = node->id;
		node = node->parent;
	}
	while (node && n < maxPathSize);

	return n;
}

bool dtStatNavMesh::closestPointToPoly(dtPolyRef ref, const float* pos, float* closest) const
{
	const dtPoly* poly = getPolyByRef(ref);
	if (!poly)
		return false;

	float closestDistSqr = FLT_MAX;

	for (int i = 2; i < (int)poly->nv; ++i)
	{
		const float* v0 = getVertex(poly->v[0]);
		const float* v1 = getVertex(poly->v[i-1]);
		const float* v2 = getVertex(poly->v[i]);
		
		float pt[3];
		closestPtPointTriangle(pt, pos, v0, v1, v2);
		float d = vdistSqr(pos, pt);
		if (d < closestDistSqr)
		{
			vcopy(closest, pt);
			closestDistSqr = d;
		}
	}
	
	return true;
}

int dtStatNavMesh::findStraightPath(const float* startPos, const float* endPos,
									const dtPolyRef* path, const int pathSize,
									float* straightPath, const int maxStraightPathSize)
{
	if (!maxStraightPathSize)
		return 0;

	if (!path[0])
		return 0;

	int straightPathSize = 0;
	
	float closestStartPos[3];
	if (!closestPointToPoly(path[0], startPos, closestStartPos))
		return 0;

	// Add start point.
	vcopy(&straightPath[straightPathSize*3], closestStartPos);
	straightPathSize++;
	if (straightPathSize >= maxStraightPathSize)
		return straightPathSize;

	float closestEndPos[3];
	if (!closestPointToPoly(path[pathSize-1], endPos, closestEndPos))
		return 0;

	float portalApex[3], portalLeft[3], portalRight[3];

	if (pathSize > 1)
	{
		vcopy(portalApex, closestStartPos);
		vcopy(portalLeft, portalApex);
		vcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;

		for (int i = 0; i < pathSize; ++i)
		{
			float left[3], right[3];
			if (i < pathSize-1)
			{
				// Next portal.
				getPortalPoints(path[i], path[i+1], left, right);
			}
			else
			{
				// End of the path.
				vcopy(left, closestEndPos);
				vcopy(right, closestEndPos);
			}

			// Right vertex.
			if (vequal(portalApex, portalRight))
			{
				vcopy(portalRight, right);
				rightIndex = i;
			}
			else
			{
				if (triArea2D(portalApex, portalRight, right) <= 0.0f)
				{
					if (triArea2D(portalApex, portalLeft, right) > 0.0f)
					{
						vcopy(portalRight, right);
						rightIndex = i;
					}
					else
					{
						vcopy(portalApex, portalLeft);
						apexIndex = leftIndex;

						if (!vequal(&straightPath[(straightPathSize-1)*3], portalApex))
						{
							vcopy(&straightPath[straightPathSize*3], portalApex);
							straightPathSize++;
							if (straightPathSize >= maxStraightPathSize)
								return straightPathSize;
						}

						vcopy(portalLeft, portalApex);
						vcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;

						// Restart
						i = apexIndex;

						continue;
					}
				}
			}

			// Left vertex.
			if (vequal(portalApex, portalLeft))
			{
				vcopy(portalLeft, left);
				leftIndex = i;
			}
			else
			{
				if (triArea2D(portalApex, portalLeft, left) >= 0.0f)
				{
					if (triArea2D(portalApex, portalRight, left) < 0.0f)
					{
						vcopy(portalLeft, left);
						leftIndex = i;
					}
					else
					{
						vcopy(portalApex, portalRight);
						apexIndex = rightIndex;

						if (!vequal(&straightPath[(straightPathSize-1)*3], portalApex))
						{
							vcopy(&straightPath[straightPathSize*3], portalApex);
							straightPathSize++;
							if (straightPathSize >= maxStraightPathSize)
								return straightPathSize;
						}

						vcopy(portalLeft, portalApex);
						vcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;

						// Restart
						i = apexIndex;

						continue;
					}
				}
			}
		}
	}

	// Add end point.
	vcopy(&straightPath[straightPathSize*3], closestEndPos);
	straightPathSize++;
	
	return straightPathSize;
}

int dtStatNavMesh::getPolyVerts(dtPolyRef ref, float* verts)
{
	const dtPoly* poly = getPolyByRef(ref);
	if (!poly)
		return 0;
	float* v = verts;
	for (int i = 0; i < (int)poly->nv; ++i)
	{
		const float* cv = &m_verts[poly->v[i]*3];
		*v++ = cv[0];
		*v++ = cv[1];
		*v++ = cv[2];
	}
	return (int)poly->nv;
}

bool dtStatNavMesh::raycast(dtPolyRef centerRef, const float* startPos, const float* endPos,
					  float& t, dtPolyRef& endRef)
{
	endRef = centerRef;
	
	if (!centerRef)
		return 0;

	dtPolyRef prevRef = centerRef;
	dtPolyRef curRef = centerRef;
	t = 0;

	float verts[DT_VERTS_PER_POLYGON*3];

	while (curRef)
	{
		// Cast ray against current polygon.
		int nv = getPolyVerts(curRef, verts);
		if (nv < 3)
		{
			// Hit bad polygon, report hit.
			return true;
		}
		
		float tmin, tmax;
		int segMin, segMax;
		if (!intersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not a polygon, keep the old t and report hit.
			return true;
		}
		// Keep track of furthest t so far.
		if (tmax > t)
			t = tmax;

		endRef = curRef;

		// Check the neighbour of this polygon.
		const dtPoly* poly = getPolyByRef(curRef);
		dtPolyRef nextRef = poly->n[segMax];
		if (!nextRef)
		{
			// No neighbour, we hit a wall.
			return true;
		}
		
		// No hit, advance to neighbour polygon.
		prevRef = curRef;
		curRef = nextRef;
	}
	
	return 0;
}


float dtStatNavMesh::findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius,
								  float* hitPos, float* hitNormal)
{
	if (!centerRef)
		return 0;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(centerRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = dtNode::OPEN;
	m_openList->push(startNode);
	
	float radiusSqr = sqr(maxRadius);
	
	hitNormal[0] = 1;
	hitNormal[1] = 0;
	hitNormal[2] = 0;
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		const dtPoly* poly = getPoly(bestNode->id-1);
		
		// Hit test walls.
		for (int i = 0, j = (int)poly->nv-1; i < (int)poly->nv; j = i++)
		{
			// Skip non-solid edges.
			if (poly->n[j]) continue;
			
			// Calc distance to the edge.
			const float* vj = getVertex(poly->v[j]);
			const float* vi = getVertex(poly->v[i]);
			float tseg;
			float distSqr = distancePtSegSqr2D(centerPos, vj, vi, tseg);

			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;
				
			// Hit wall, update radius.
			radiusSqr = distSqr;
			// Calculate hit pos.
			hitPos[0] = vj[0] + (vi[0] - vj[0])*tseg;
			hitPos[1] = vj[1] + (vi[1] - vj[1])*tseg;
			hitPos[2] = vj[2] + (vi[2] - vj[2])*tseg;
		}

		// Check to see if teh circle expands to one of the neighbours and expand.
		for (int i = 0, j = (int)poly->nv-1; i < (int)poly->nv; j = i++)
		{
			// Skip solid edges.
			if (!poly->n[j]) continue;
			
			// Expand to neighbour if not visited yet.
			dtPolyRef neighbour = poly->n[j];
			
			// Skip parent node.
			if (bestNode->parent && bestNode->parent->id == neighbour)
				continue;
				
			// Calc distance to the edge.
			const float* vj = getVertex(poly->v[j]);
			const float* vi = getVertex(poly->v[i]);
			float tseg;
			float distSqr = distancePtSegSqr2D(centerPos, vj, vi, tseg);
			
			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;
				
			dtNode newNode;
			newNode.parent = bestNode;
			newNode.id = neighbour;
			newNode.cost = bestNode->cost + 1; // Depth
			newNode.total = bestNode->total + getCost(newNode.parent->id, newNode.id);
			
			dtNode* actualNode = m_nodePool->getNode(newNode.id);
			if (!actualNode)
				continue;

			if (!((actualNode->flags & dtNode::OPEN) && newNode.total > actualNode->total) &&
				!((actualNode->flags & dtNode::CLOSED) && newNode.total > actualNode->total))
			{
				actualNode->flags &= ~dtNode::CLOSED;
				actualNode->parent = newNode.parent;
				actualNode->cost = newNode.cost;
				actualNode->total = newNode.total;
				
				if (actualNode->flags & dtNode::OPEN)
				{
					m_openList->modify(actualNode);
				}
				else
				{
					actualNode->flags = dtNode::OPEN;
					m_openList->push(actualNode);
				}
			}
		}
	}

	// Calc hit normal.
	vsub(hitNormal, centerPos, hitPos);
	vnormalize(hitNormal);
	
	return sqrtf(radiusSqr);
}

int dtStatNavMesh::findPolysAround(dtPolyRef centerRef, const float* centerPos, float radius,
								   dtPolyRef* resultRef, dtPolyRef* resultParent,
								   unsigned short* resultCost, unsigned short* resultDepth,
								   const int maxResult)
{
	if (!centerRef)
		return 0;

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(centerRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = dtNode::OPEN;
	m_openList->push(startNode);

	unsigned n = 0;
	if (n < maxResult)
	{
		if (resultRef)
			resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		if (resultCost)
			resultCost[n] = 0;
		if (resultDepth)
			resultDepth[n] = 0;
		++n;
	}

	const float radiusSqr = sqr(radius);

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		const dtPoly* poly = getPoly(bestNode->id-1);
		for (unsigned i = 0, j = (int)poly->nv-1; i < (int)poly->nv; j=i++)
		{
			dtPolyRef neighbour = poly->n[j];

			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->parent && bestNode->parent->id == neighbour)
					continue;
					
				// Calc distance to the edge.
				const float* vj = getVertex(poly->v[j]);
				const float* vi = getVertex(poly->v[i]);
				float tseg;
				float distSqr = distancePtSegSqr2D(centerPos, vj, vi, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;
				
				dtNode newNode;
				newNode.parent = bestNode;
				newNode.id = neighbour;
				newNode.cost = bestNode->cost + 1; // Depth
				newNode.total = bestNode->total + getCost(newNode.parent->id, newNode.id);

				dtNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;

				if (!((actualNode->flags & dtNode::OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & dtNode::CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= ~dtNode::CLOSED;
					actualNode->parent = newNode.parent;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;

					if (actualNode->flags & dtNode::OPEN)
					{
						m_openList->modify(actualNode);
					}
					else
					{
						if (n < maxResult)
						{
							if (resultRef)
								resultRef[n] = actualNode->id;
							if (resultParent)
								resultParent[n] = actualNode->parent->id;
							if (resultCost)
								resultCost[n] = actualNode->total;
							if (resultDepth)
								resultDepth[n] = actualNode->cost;
							++n;
						}
						actualNode->flags = dtNode::OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
	}

	return n;
}

inline bool checkOverlapBox(const unsigned short amin[3], const unsigned short amax[3],
							const unsigned short bmin[3], const unsigned short bmax[3])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Returns polygons which are withing certain radius from the query location.
int dtStatNavMesh::queryPolygons(const float* center, const float* extents,
								 unsigned short* ids, const int maxIds)
{
	const dtBVNode* node = &m_bvtree[0];
	const dtBVNode* end = &m_bvtree[m_header->nnodes];

	// Calculate quantized box
	const float ics = 1.0f / m_header->cs;
	unsigned short bmin[3], bmax[3];
	// Clamp query box to world box.
	float minx = clamp(center[0] - extents[0], m_header->bmin[0], m_header->bmax[0]) - m_header->bmin[0];
	float miny = clamp(center[1] - extents[1], m_header->bmin[1], m_header->bmax[1]) - m_header->bmin[1];
	float minz = clamp(center[2] - extents[2], m_header->bmin[2], m_header->bmax[2]) - m_header->bmin[2];
	float maxx = clamp(center[0] + extents[0], m_header->bmin[0], m_header->bmax[0]) - m_header->bmin[0];
	float maxy = clamp(center[1] + extents[1], m_header->bmin[1], m_header->bmax[1]) - m_header->bmin[1];
	float maxz = clamp(center[2] + extents[2], m_header->bmin[2], m_header->bmax[2]) - m_header->bmin[2];
	// Quantize
	bmin[0] = (unsigned short)(ics * minx) & 0xfffe;
	bmin[1] = (unsigned short)(ics * miny) & 0xfffe;
	bmin[2] = (unsigned short)(ics * minz) & 0xfffe;
	bmax[0] = (unsigned short)(ics * maxx + 1) | 1;
	bmax[1] = (unsigned short)(ics * maxy + 1) | 1;
	bmax[2] = (unsigned short)(ics * maxz + 1) | 1;
	
	// Traverse tree
	unsigned n = 0;
	while (node < end)
	{
		bool overlap = checkOverlapBox(bmin, bmax, node->bmin, node->bmax);
		bool isLeafNode = node->i >= 0;
		
		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = (unsigned short)node->i;
				n++;
			}
		}
		
		if (overlap || isLeafNode)
			node++;
		else
		{
			const int escapeIndex = -node->i;
			node += escapeIndex;
		}
	}
	
	return n;
}

dtPolyRef dtStatNavMesh::findNearestPoly(const float* center, const float* extents)
{
	// Get nearby polygons from proximity grid.
	unsigned short polys[128];
	int npolys = queryPolygons(center, extents, polys, 128);

	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < npolys; ++i)
	{
		dtPolyRef ref = (dtPolyRef)polys[i];
		float closest[3];
		if (!closestPointToPoly(ref, center, closest))
			continue;
		float d = vdistSqr(center, closest);
		if (d < nearestDistanceSqr)
		{
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}

	return nearest;
}

bool dtStatNavMesh::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right)
{
	const dtPoly* fromPoly = getPolyByRef(from);
	if (!fromPoly)
		return false;

	// Find common edge between the polygons and returns the segment end points.
	for (unsigned i = 0, j = (int)fromPoly->nv - 1; i < (int)fromPoly->nv; j = i++)
	{
		unsigned short neighbour = fromPoly->n[j];
		if (neighbour == to)
		{
			vcopy(left, getVertex(fromPoly->v[j]));
			vcopy(right, getVertex(fromPoly->v[i]));
			return true;
		}
	}

	return false;
}

bool dtStatNavMesh::isInOpenList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;
	return m_nodePool->findNode(ref) != 0;
}

int dtStatNavMesh::getMemUsed() const
{
	if (!m_nodePool || ! m_openList)
		return 0;
	return sizeof(*this) + m_dataSize +
			m_nodePool->getMemUsed() +
			m_openList->getMemUsed();
}	
