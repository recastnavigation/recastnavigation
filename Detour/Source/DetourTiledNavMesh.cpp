#include "DetourTiledNavMesh.h"

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

static void calcPolyCenter(float* tc, const dtTilePoly* p, const float* verts)
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

inline bool overlapRects(const float* amin, const float* amax,
						 const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	return overlap;
}

inline bool overlapBounds(const float* amin, const float* amax,
						  const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline int opposite(int side) { return (side+2) & 0x3; }

static void calcBounds(const float* va, const float* vb,
					   float* bmin, float* bmax,
					   int side, float padx, float pady)
{
	if ((side&1) == 0)
	{
		bmin[0] = min(va[2],vb[2]) + padx;
		bmin[1] = min(va[1],vb[1]) - pady;
		bmax[0] = max(va[2],vb[2]) - padx;
		bmax[1] = max(va[1],vb[1]) + pady;
	}
	else
	{
		bmin[0] = min(va[0],vb[0]) + padx;
		bmin[1] = min(va[1],vb[1]) - pady;
		bmax[0] = max(va[0],vb[0]) - padx;
		bmax[1] = max(va[1],vb[1]) + pady;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
struct dtTileNode
{
	enum dtTileNodeFlags
	{
		OPEN = 0x01,
		CLOSED = 0x02,
	};
	dtTileNode* parent;
	float cost;
	float total;
	unsigned int id;
	unsigned char flags;	// TODO: merge to id or parent?
};

class dtTileNodePool
{
public:
	dtTileNodePool(int maxNodes, int hashSize);
	~dtTileNodePool();
	inline void operator=(const dtTileNodePool&) {}
	void clear();
	dtTileNode* getNode(unsigned short id);
	const dtTileNode* findNode(unsigned short id) const;
	
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtTileNode)*m_maxNodes +
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
	
	dtTileNode* m_nodes;
	unsigned short* m_first;
	unsigned short* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

dtTileNodePool::dtTileNodePool(int maxNodes, int hashSize) :
	m_maxNodes(maxNodes),
	m_hashSize(hashSize),
	m_nodes(0),
	m_first(0),
	m_next(0)
{
	m_nodes = new dtTileNode[m_maxNodes];
	m_next = new unsigned short[m_maxNodes];
	m_first = new unsigned short[hashSize];
	memset(m_first, 0xff, sizeof(unsigned short)*m_hashSize);
	memset(m_next, 0xff, sizeof(unsigned short)*m_maxNodes);
}

dtTileNodePool::~dtTileNodePool()
{
	delete [] m_nodes;
	delete [] m_next;
	delete [] m_first;
}

void dtTileNodePool::clear()
{
	memset(m_first, 0xff, sizeof(unsigned short)*m_hashSize);
	m_nodeCount = 0;
}

const dtTileNode* dtTileNodePool::findNode(unsigned short id) const
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

dtTileNode* dtTileNodePool::getNode(unsigned short id)
{
	unsigned int bucket = hashint((unsigned int)id) & (m_hashSize-1);
	unsigned short i = m_first[bucket];
	dtTileNode* node = 0;
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
class dtTileNodeQueue
{
public:
	dtTileNodeQueue(int n);
	~dtTileNodeQueue();
	inline void operator=(dtTileNodeQueue&) {}
	
	inline void clear()
	{
		m_size = 0;
	}
	
	inline dtTileNode* top()
	{
		return m_heap[0];
	}
	
	inline dtTileNode* pop()
	{
		dtTileNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	inline void push(dtTileNode* node)
	{
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	inline void modify(dtTileNode* node)
	{
		for (int i = 0; i < m_size; ++i)
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
		sizeof(dtTileNode*)*(m_capacity+1);
	}
	
	
private:
	void bubbleUp(int i, dtTileNode* node);
	void trickleDown(int i, dtTileNode* node);
	
	dtTileNode** m_heap;
	const int m_capacity;
	int m_size;
};		

dtTileNodeQueue::dtTileNodeQueue(int n) :
	m_capacity(n),
	m_size(0),
	m_heap(0)
{
	m_heap = new dtTileNode*[m_capacity+1];
}

dtTileNodeQueue::~dtTileNodeQueue()
{
	delete [] m_heap;
}

void dtTileNodeQueue::bubbleUp(int i, dtTileNode* node)
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

void dtTileNodeQueue::trickleDown(int i, dtTileNode* node)
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
dtTiledNavMesh::dtTiledNavMesh() :
	m_tileSize(0),
	m_portalHeight(0),
	m_nextFree(0),
	m_tmpLinks(0),
	m_ntmpLinks(0),
	m_nodePool(0),
	m_openList(0)
{
}

dtTiledNavMesh::~dtTiledNavMesh()
{
	// TODO! the mesh should not handle the tile memory!
	for (int i = 0; i < DT_MAX_TILES; ++i)
	{
		if (m_tiles[i].header)
		{
			delete [] (unsigned char*)m_tiles[i].header;
			m_tiles[i].header = 0;
		}
	}
	
	delete [] m_tmpLinks;

	delete m_nodePool;
	delete m_openList;
}
		
bool dtTiledNavMesh::init(const float* orig, float tileSize, float portalHeight)
{
	vcopy(m_orig, orig);
	m_tileSize = tileSize;
	m_portalHeight = portalHeight;
	
	// Init tiles
	memset(m_tiles, 0, sizeof(dtTile)*DT_MAX_TILES);
	memset(m_posLookup, 0, sizeof(dtTile*)*DT_TILE_LOOKUP_SIZE);
	m_nextFree = 0;
	for (int i = DT_MAX_TILES-1; i >= 0; --i)
	{
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}

	m_nodePool = new dtTileNodePool(2048, 256);
	if (!m_nodePool)
		return false;
	
	m_openList = new dtTileNodeQueue(2048);
	if (!m_openList)
		return false;
	
	return true;
}

int dtTiledNavMesh::getPolyNeighbours(dtTilePolyRef ref, dtTilePolyRef* nei, int maxNei) const
{
	int salt, it, ip;
	decodeId(ref, salt, it, ip);
	if (it >= DT_MAX_TILES) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const dtTileHeader* h = m_tiles[it].header;

	if (ip >= h->npolys) return 0;
	const dtTilePoly* poly = &h->polys[ip];

	int n = 0;
	for (int i = 0; i < poly->nlinks; ++i)
		if (n < maxNei) nei[n++] = h->links[poly->links+i].ref;
	
	return n;
}

int dtTiledNavMesh::findConnectingPolys(const float* va, const float* vb,
										dtTile* tile, int side,
										dtTilePolyRef* con, float* conarea, int maxcon)
{
	if (!tile) return 0;
	dtTileHeader* h = tile->header;
	
	float amin[2], amax[2];
	calcBounds(va,vb, amin,amax, side, 0.01f, m_portalHeight);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = 0x8000 | (unsigned short)side;
	int n = 0;
	
	dtTilePolyRef base = getTileId(tile);
	
	for (int i = 0; i < h->npolys; ++i)
	{
		dtTilePoly* poly = &h->polys[i];
		for (int j = 0; j < poly->nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->n[j] != m) continue;
			// Check if the segments touch.
			const float* vc = &h->verts[poly->v[j]*3];
			const float* vd = &h->verts[poly->v[(j+1) % (int)poly->nv]*3];
			calcBounds(vc,vd, bmin,bmax, side, 0.01f, m_portalHeight);
			if (!overlapRects(amin,amax, bmin,bmax)) continue;
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = max(amin[0], bmin[0]);
				conarea[n*2+1] = min(amax[0], bmax[0]);
				con[n] = base | (unsigned int)i;
				n++;
			}
			break;
		}
	}
	return n;
}

void dtTiledNavMesh::removeExtLinks(dtTile* tile, int side)
{
	if (!tile) return;
	dtTileHeader* h = tile->header;
	
	// Remove links pointing to 'side' and compact the links array. 
	dtTileLink* pool = m_tmpLinks;
	int nlinks = 0;
	for (int i = 0; i < h->npolys; ++i)
	{
		dtTilePoly* poly = &h->polys[i];
		int plinks = nlinks;
		int nplinks = 0;		
		for (int j = 0; j < poly->nlinks; ++j)
		{
			dtTileLink* link = &h->links[poly->links+j];
			if ((int)link->side != side)
			{
				if (nlinks < h->maxlinks)
				{
					dtTileLink* dst = &pool[nlinks++];
					memcpy(dst, link, sizeof(dtTileLink));
					nplinks++;
				}
			}
		}
		poly->links = plinks;
		poly->nlinks = nplinks;
	}
	h->nlinks = nlinks;
	if (h->nlinks)
		memcpy(h->links, m_tmpLinks, sizeof(dtTileLink)*nlinks);
}

void dtTiledNavMesh::buildExtLinks(dtTile* tile, dtTile* target, int side)
{
	if (!tile) return;
	dtTileHeader* h = tile->header;
	
	// Remove links pointing to 'side' and compact the links array. 
	dtTileLink* pool = m_tmpLinks;
	int nlinks = 0;
	for (int i = 0; i < h->npolys; ++i)
	{
		dtTilePoly* poly = &h->polys[i];
		int plinks = nlinks;
		int nplinks = 0;
		// Copy internal and other external links.
		for (int j = 0; j < poly->nlinks; ++j)
		{
			dtTileLink* link = &h->links[poly->links+j];
			if ((int)link->side != side)
			{
				if (nlinks < h->maxlinks)
				{
					dtTileLink* dst = &pool[nlinks++];
					memcpy(dst, link, sizeof(dtTileLink));
					nplinks++;
				}
			}
		}
		// Create new links.
		unsigned short m = 0x8000 | (unsigned short)side;
		for (int j = 0; j < poly->nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->n[j] != m) continue;
			
			// Create new links
			const float* va = &h->verts[poly->v[j]*3];
			const float* vb = &h->verts[poly->v[(j+1)%(int)poly->nv]*3];
			dtTilePolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, opposite(side), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				if (nlinks < h->maxlinks)
				{
					dtTileLink* link = &pool[nlinks++];
					link->ref = nei[k];
					link->p = (unsigned short)i;
					link->e = (unsigned char)j;
					link->side = (unsigned char)side;
					link->bmin = neia[k*2+0];
					link->bmax = neia[k*2+1];
					nplinks++;
				}
			}
		}
		
		poly->links = plinks;
		poly->nlinks = nplinks;
	}
	h->nlinks = nlinks;
	if (h->nlinks)
		memcpy(h->links, m_tmpLinks, sizeof(dtTileLink)*nlinks);
}

void dtTiledNavMesh::buildIntLinks(dtTile* tile)
{
	if (!tile) return;
	dtTileHeader* h = tile->header;

	dtTilePolyRef base = getTileId(tile);
	dtTileLink* pool = h->links;
	int nlinks = 0;
	for (int i = 0; i < h->npolys; ++i)
	{
		dtTilePoly* poly = &h->polys[i];
		poly->links = nlinks;
		poly->nlinks = 0;
		for (int j = 0; j < poly->nv; ++j)
		{
			// Skip hard and non-internal edges.
			if (poly->n[j] == 0 || (poly->n[j] & 0x8000)) continue;
			
			if (nlinks < h->maxlinks)
			{
				dtTileLink* link = &pool[nlinks++];
				link->ref = base | (unsigned int)(poly->n[j]-1);
				link->p = (unsigned short)i;
				link->e = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				poly->nlinks++;
			}
		}
	}
	h->nlinks = nlinks;
}

inline int computeTileHash(int x, int y)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & (DT_TILE_LOOKUP_SIZE-1));
}

bool dtTiledNavMesh::addTile(int x, int y, unsigned char* data, int dataSize)
{
	// Remove any old tile at this location.
	removeTile(x,y);
	// Make sure there is enough space for new tile.
	if (!m_nextFree)
		return false;
	// Make sure the data is in right format.
	dtTileHeader* header = (dtTileHeader*)data;
	if (header->magic != DT_TILE_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_TILE_NAVMESH_VERSION)
		return false;
	
	// Make sure the tmp link array is large enough.
	if (header->maxlinks > m_ntmpLinks)
	{
		m_ntmpLinks = header->maxlinks;
		delete [] m_tmpLinks;
		m_tmpLinks = 0;
		m_tmpLinks = new dtTileLink[m_ntmpLinks];
	}
	if (!m_tmpLinks)
		return false;
	
	// Allocate a tile.
	dtTile* tile = m_nextFree;
	m_nextFree = tile->next;
	tile->next = 0;

	// Insert tile into the position lut.
	int h = computeTileHash(x,y);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = sizeof(dtTileHeader);
	const int vertsSize = sizeof(float)*3*header->nverts;
	const int polysSize = sizeof(dtTilePoly)*header->npolys;
	header->verts = (float*)(data + headerSize);
	header->polys = (dtTilePoly*)(data + headerSize + vertsSize);
	header->links = (dtTileLink*)(data + headerSize + vertsSize + polysSize);

	// Init tile.
	tile->header = header;
	tile->x = x;
	tile->y = y;

	buildIntLinks(tile);

	// Create connections connections.
	for (int i = 0; i < 4; ++i)
	{
		dtTile* nei = getNeighbourTile(x,y,i);
		tile->header->nei[i] = nei;
		if (tile->header->nei[i])
		{
			nei->header->nei[opposite(i)] = tile;
			buildExtLinks(tile, nei, i);
			buildExtLinks(nei, tile, opposite(i));
		}
	}
	
	return true;
}

dtTile* dtTiledNavMesh::getTile(int x, int y)
{
	// Find tile based on hash.
	int h = computeTileHash(x,y);
	dtTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->x == x && tile->y == y)
			return tile;
		tile = tile->next;
	}
	return 0;
}

dtTile* dtTiledNavMesh::getNeighbourTile(int x, int y, int side)
{
	switch (side)
	{
	case 0: x++; break;
	case 1: y++; break;
	case 2: x--; break;
	case 3: y--; break;
	};
	return getTile(x,y);
}

bool dtTiledNavMesh::removeTile(int x, int y)
{
	dtTile* tile = getTile(x, y);
	if (!tile)
		return false;

	// Remove connections to neighbour tiles.
	for (int i = 0; i < 4; ++i)
	{
		dtTile* nei = getNeighbourTile(x,y,i);
		if (!nei) continue;
		nei->header->nei[opposite(i)] = 0;
		removeExtLinks(nei, opposite(i));
	}

	// Reset tile.
	unsigned char* data = (unsigned char*)tile->header;
	tile->header = 0;
	tile->x = -1;
	tile->y = -1;
	tile->salt++;

	// TODO! the mesh should not handle the tile memory!
	delete [] data;
	
	return true;
}





bool dtTiledNavMesh::closestPointToPoly(dtTilePolyRef ref, const float* pos, float* closest) const
{
	int salt, it, ip;
	decodeId(ref, salt, it, ip);
	if (it >= DT_MAX_TILES) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtTileHeader* h = m_tiles[it].header;

	if (ip >= h->npolys) return false;
	const dtTilePoly* poly = &h->polys[ip];
	
	float closestDistSqr = FLT_MAX;
	
	for (int i = 2; i < (int)poly->nv; ++i)
	{
		const float* v0 = &h->verts[poly->v[0]*3];
		const float* v1 = &h->verts[poly->v[i-1]*3];
		const float* v2 = &h->verts[poly->v[i]*3];
		
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

dtTilePolyRef dtTiledNavMesh::findNearestPoly(const float* center, const float* extents)
{
	// Get nearby polygons from proximity grid.
	dtTilePolyRef polys[128];
	int npolys = queryPolygons(center, extents, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	dtTilePolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < npolys; ++i)
	{
		dtTilePolyRef ref = polys[i];
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

dtTilePolyRef dtTiledNavMesh::getTileId(dtTile* tile)
{
	if (!tile) return 0;
	const unsigned int it = tile - m_tiles;
	return encodeId(tile->salt, it, 0);
}

int dtTiledNavMesh::queryTilePolygons(dtTile* tile,
									  const float* qmin, const float* qmax,
									  dtTilePolyRef* polys, const int maxPolys)
{
	float bmin[3], bmax[3];
	const dtTileHeader* header = tile->header;
	int n = 0;
	dtTilePolyRef base = getTileId(tile);
	for (int i = 0; i < header->npolys; ++i)
	{
		// Calc polygon bounds.
		dtTilePoly* p = &header->polys[i];
		const float* v = &header->verts[p->v[0]*3];
		vcopy(bmin, v);
		vcopy(bmax, v);
		for (int j = 1; j < p->nv; ++j)
		{
			v = &header->verts[p->v[j]*3];
			vmin(bmin, v);
			vmax(bmax, v);
		}
		if (overlapBounds(qmin,qmax, bmin,bmax))
		{
			if (n < maxPolys)
				polys[n++] = base | (dtTilePolyRef)i;
		}
	}
	return n;
}

int dtTiledNavMesh::queryPolygons(const float* center, const float* extents,
								  dtTilePolyRef* polys, const int maxPolys)
{
	float bmin[3], bmax[3];
	bmin[0] = center[0] - extents[0];
	bmin[1] = center[1] - extents[1];
	bmin[2] = center[2] - extents[2];
	
	bmax[0] = center[0] + extents[0];
	bmax[1] = center[1] + extents[1];
	bmax[2] = center[2] + extents[2];
	
	// Find tiles the query touches.
	const int minx = (int)floorf((bmin[0]-m_orig[0]) / m_tileSize);
	const int maxx = (int)ceilf((bmax[0]-m_orig[0]) / m_tileSize);

	const int miny = (int)floorf((bmin[2]-m_orig[2]) / m_tileSize);
	const int maxy = (int)ceilf((bmax[2]-m_orig[2]) / m_tileSize);

	int n = 0;
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			dtTile* tile = getTile(x,y);
			if (!tile) continue;
			n += queryTilePolygons(tile, bmin, bmax, polys+n, maxPolys-n);
			if (n >= maxPolys) return n;
		}
	}

	return n;
}

float dtTiledNavMesh::getCost(dtTilePolyRef prev, dtTilePolyRef from, dtTilePolyRef to) const
{
	int salt, it, ip;
	if (prev) from = prev;
	// The API input has been cheked already, skip checking internal data.
	decodeId(from, salt, it, ip);
	const dtTileHeader* fromHeader = m_tiles[it].header;
	const dtTilePoly* fromPoly = &fromHeader->polys[ip];
	decodeId(to, salt, it, ip);
	const dtTileHeader* toHeader = m_tiles[it].header;
	const dtTilePoly* toPoly = &toHeader->polys[ip];
	
	float fromPc[3], toPc[3];
	calcPolyCenter(fromPc, fromPoly, fromHeader->verts);
	calcPolyCenter(toPc, toPoly, toHeader->verts);
	
	float dx = fromPc[0]-toPc[0];
	float dy = fromPc[1]-toPc[1];
	float dz = fromPc[2]-toPc[2];
	
	return sqrtf(dx*dx + dy*dy + dz*dz);
}

float dtTiledNavMesh::getHeuristic(dtTilePolyRef from, dtTilePolyRef to) const
{
	int salt, it, ip;
	// The API input has been cheked already, skip checking internal data.
	decodeId(from, salt, it, ip);
	const dtTileHeader* fromHeader = m_tiles[it].header;
	const dtTilePoly* fromPoly = &fromHeader->polys[ip];
	decodeId(to, salt, it, ip);
	const dtTileHeader* toHeader = m_tiles[it].header;
	const dtTilePoly* toPoly = &toHeader->polys[ip];
	
	float fromPc[3], toPc[3];
	calcPolyCenter(fromPc, fromPoly, fromHeader->verts);
	calcPolyCenter(toPc, toPoly, toHeader->verts);
	
	float dx = fromPc[0]-toPc[0];
	float dy = fromPc[1]-toPc[1];
	float dz = fromPc[2]-toPc[2];
	
	return sqrtf(dx*dx + dy*dy + dz*dz) * 2.0f;
}

int dtTiledNavMesh::findPath(dtTilePolyRef startRef, dtTilePolyRef endRef,
							 dtTilePolyRef* path, const int maxPathSize)
{
	if (!startRef || !endRef)
		return 0;
	
	if (!maxPathSize)
		return 0;
	
	if (!getPolyByRef(startRef) || !getPolyByRef(endRef))
		return 0;
	
	if (startRef == endRef)
	{
		path[0] = startRef;
		return 1;
	}
	
	if (!m_nodePool || !m_openList)
		return 0;
		
	m_nodePool->clear();
	m_openList->clear();
	
	dtTileNode* startNode = m_nodePool->getNode(startRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = getHeuristic(startRef, endRef);
	startNode->id = startRef;
	startNode->flags = dtTileNode::OPEN;
	m_openList->push(startNode);
	
	dtTileNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;
	while (!m_openList->empty())
	{
		dtTileNode* bestNode = m_openList->pop();
		
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		// Get poly and tile.
		int salt, it, ip;
		decodeId(bestNode->id, salt, it, ip);
		// The API input has been cheked already, skip checking internal data.
		const dtTileHeader* h = m_tiles[it].header;
		const dtTilePoly* poly = &h->polys[ip];
		
		for (int i = 0; i < poly->nlinks; ++i)
		{
			dtTilePolyRef neighbour = h->links[poly->links+i].ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->parent && bestNode->parent->id == neighbour)
					continue;
				
				dtTileNode newNode;
				newNode.parent = bestNode;
				newNode.id = neighbour;
				newNode.cost = bestNode->cost + getCost(newNode.parent->parent ? newNode.parent->parent->id : 0, newNode.parent->id, newNode.id);
				float h = getHeuristic(newNode.id, endRef);
				newNode.total = newNode.cost + h;
				
				dtTileNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;
				
				if (!((actualNode->flags & dtTileNode::OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & dtTileNode::CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= dtTileNode::CLOSED;
					actualNode->parent = newNode.parent;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;
					
					if (h < lastBestNodeCost)
					{
						lastBestNodeCost = h;
						lastBestNode = actualNode;
					}
					
					if (actualNode->flags & dtTileNode::OPEN)
					{
						m_openList->modify(actualNode);
					}
					else
					{
						actualNode->flags = dtTileNode::OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
	}
	
	// Reverse the path.
	dtTileNode* prev = 0;
	dtTileNode* node = lastBestNode;
	do
	{
		dtTileNode* next = node->parent;
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

int dtTiledNavMesh::findStraightPath(const float* startPos, const float* endPos,
									 const dtTilePolyRef* path, const int pathSize,
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
				if (!getPortalPoints(path[i], path[i+1], left, right))
				{
					if (!closestPointToPoly(path[i], endPos, closestEndPos))
						return 0;
					vcopy(&straightPath[straightPathSize*3], closestEndPos);
					straightPathSize++;
					return straightPathSize;
				}
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

// Returns portal points between two polygons.
bool dtTiledNavMesh::getPortalPoints(dtTilePolyRef from, dtTilePolyRef to, float* left, float* right) const
{
	int salt, it, ip;
	decodeId(from, salt, it, ip);
	if (it >= DT_MAX_TILES) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	if (ip >= m_tiles[it].header->npolys) return false;
	const dtTileHeader* fromHeader = m_tiles[it].header;
	const dtTilePoly* fromPoly = &fromHeader->polys[ip];

	for (int i = 0; i < fromPoly->nlinks; ++i)
	{
		const dtTileLink* link = &fromHeader->links[fromPoly->links+i];
		if (link->ref == to)
		{
			// Find portal vertices.
			const int v0 = fromPoly->v[link->e];
			const int v1 = fromPoly->v[(link->e+1) % fromPoly->nv];
			vcopy(left, &fromHeader->verts[v0*3]);
			vcopy(right, &fromHeader->verts[v1*3]);
			// If the link is at tile boundary, clamp the vertices to
			// the link width.
			if (link->side == 0 || link->side == 2)
			{
				left[2] = max(left[2],link->bmin);
				left[2] = min(left[2],link->bmax);
				right[2] = max(right[2],link->bmin);
				right[2] = min(right[2],link->bmax);
			}
			else if (link->side == 1 || link->side == 3)
			{
				left[0] = max(left[0],link->bmin);
				left[0] = min(left[0],link->bmax);
				right[0] = max(right[0],link->bmin);
				right[0] = min(right[0],link->bmax);
			}
			return true;
		}
	}
	return false;
}

int dtTiledNavMesh::raycast(dtTilePolyRef centerRef, const float* startPos, const float* endPos,
							float& t, dtTilePolyRef* path, const int pathSize)
{
	t = 0;
	
	if (!centerRef || !getPolyByRef(centerRef))
		return 0;
	
	dtTilePolyRef curRef = centerRef;
	float verts[DT_TILE_VERTS_PER_POLYGON*3];	
	int n = 0;
	
	while (curRef)
	{
		// Cast ray against current polygon.
		
		// The API input has been cheked already, skip checking internal data.
		int salt, it, ip;
		decodeId(curRef, salt, it, ip);
		const dtTileHeader* h = m_tiles[it].header;
		const dtTilePoly* poly = &h->polys[ip];

		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->nv; ++i)
		{
			vcopy(&verts[nv*3], &h->verts[poly->v[i]*3]);
			nv++;
		}		
		if (nv < 3)
		{
			// Hit bad polygon, report hit.
			return n;
		}
		
		float tmin, tmax;
		int segMin, segMax;
		if (!intersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			return n;
		}
		// Keep track of furthest t so far.
		if (tmax > t)
			t = tmax;

		if (n < pathSize)
			path[n++] = curRef;
		
		// Follow neighbours.
		dtTilePolyRef nextRef = 0;
		for (int i = 0; i < poly->nlinks; ++i)
		{
			const dtTileLink* link = &h->links[poly->links+i];
			if ((int)link->e == segMax)
			{
				// If the link is internal, just return the ref.
				if (link->side == 0xff)
				{
					nextRef = link->ref;
					break;
				}
				// If the link is at tile boundary,
				// Check that the intersection lies inside the portal.
				if (link->side == 0 || link->side == 2)
				{
					// Find Z intersection.
					float z = startPos[2] + (endPos[2]-startPos[2])*tmax;
					if (z >= link->bmin && z <= link->bmax)
					{
						nextRef = link->ref;
						break;
					}
				}
				else if (link->side == 1 || link->side == 3)
				{
					// Find X intersection.
					float x = startPos[0] + (endPos[0]-startPos[0])*tmax;
					if (x >= link->bmin && x <= link->bmax)
					{
						nextRef = link->ref;
						break;
					}
				}
			}
		}
		
		if (!nextRef)
		{
			// No neighbour, we hit a wall.
			return n;
		}
		
		// No hit, advance to neighbour polygon.
		curRef = nextRef;
	}
	
	return n;
}

int dtTiledNavMesh::findPolysAround(dtTilePolyRef centerRef, const float* centerPos, float radius,
									dtTilePolyRef* resultRef, dtTilePolyRef* resultParent,
									float* resultCost, unsigned short* resultDepth,
									const int maxResult)
{
	if (!centerRef) return 0;
	if (!getPolyByRef(centerRef)) return 0;
	if (!m_nodePool || !m_openList) return 0;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtTileNode* startNode = m_nodePool->getNode(centerRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = dtTileNode::OPEN;
	m_openList->push(startNode);
	
	int n = 0;
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
		dtTileNode* bestNode = m_openList->pop();

		// Get poly and tile.
		int salt, it, ip;
		decodeId(bestNode->id, salt, it, ip);
		// The API input has been cheked already, skip checking internal data.
		const dtTileHeader* h = m_tiles[it].header;
		const dtTilePoly* poly = &h->polys[ip];
		
		for (int i = 0; i < poly->nlinks; ++i)
		{
			const dtTileLink* link = &h->links[poly->links+i];
			dtTilePolyRef neighbour = link->ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->parent && bestNode->parent->id == neighbour)
					continue;
				
				// Calc distance to the edge.
				const float* va = &h->verts[poly->v[link->e]*3];
				const float* vb = &h->verts[poly->v[(link->e+1)%poly->nv]*3];
				float tseg;
				float distSqr = distancePtSegSqr2D(centerPos, va, vb, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;
				
				dtTileNode newNode;
				newNode.parent = bestNode;
				newNode.id = neighbour;
				newNode.cost = bestNode->cost + 1; // Depth
				newNode.total = bestNode->total + getCost(newNode.parent->parent ? newNode.parent->parent->id : 0, newNode.parent->id, newNode.id);
				
				dtTileNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;
				
				if (!((actualNode->flags & dtTileNode::OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & dtTileNode::CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= ~dtTileNode::CLOSED;
					actualNode->parent = newNode.parent;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;
					
					if (actualNode->flags & dtTileNode::OPEN)
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
								resultDepth[n] = (unsigned short)actualNode->cost;
							++n;
						}
						actualNode->flags = dtTileNode::OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
	}
	
	return n;
}

float dtTiledNavMesh::findDistanceToWall(dtTilePolyRef centerRef, const float* centerPos, float maxRadius,
						 float* hitPos, float* hitNormal)
{
	if (!centerRef) return 0;
	if (!getPolyByRef(centerRef)) return 0;
	if (!m_nodePool || !m_openList) return 0;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtTileNode* startNode = m_nodePool->getNode(centerRef);
	startNode->parent = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = dtTileNode::OPEN;
	m_openList->push(startNode);
	
	float radiusSqr = sqr(maxRadius);
	
	while (!m_openList->empty())
	{
		dtTileNode* bestNode = m_openList->pop();
		
		// Get poly and tile.
		int salt, it, ip;
		decodeId(bestNode->id, salt, it, ip);
		// The API input has been cheked already, skip checking internal data.
		const dtTileHeader* h = m_tiles[it].header;
		const dtTilePoly* poly = &h->polys[ip];
		
		// Hit test walls.
		for (int i = 0, j = (int)poly->nv-1; i < (int)poly->nv; j = i++)
		{
			// Skip non-solid edges.
			if (poly->n[j] & 0x8000)
			{
				// Tile border.
				bool solid = true;
				for (int i = 0; i < poly->nlinks; ++i)
				{
					const dtTileLink* link = &h->links[poly->links+i];
					if (link->e == j && link->ref != 0)
					{
						solid = false;
						break;
					}
				}
				if (!solid) continue;
			}
			else if (poly->n[j])
			{
				// Internal edge
				continue;
			}
			
			// Calc distance to the edge.
			const float* vj = &h->verts[poly->v[j]*3];
			const float* vi = &h->verts[poly->v[i]*3];
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
		
		for (int i = 0; i < poly->nlinks; ++i)
		{
			const dtTileLink* link = &h->links[poly->links+i];
			dtTilePolyRef neighbour = link->ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->parent && bestNode->parent->id == neighbour)
					continue;
				
				// Calc distance to the edge.
				const float* va = &h->verts[poly->v[link->e]*3];
				const float* vb = &h->verts[poly->v[(link->e+1)%poly->nv]*3];
				float tseg;
				float distSqr = distancePtSegSqr2D(centerPos, va, vb, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;
				
				dtTileNode newNode;
				newNode.parent = bestNode;
				newNode.id = neighbour;
				newNode.cost = bestNode->cost + 1; // Depth
				newNode.total = bestNode->total + getCost(newNode.parent->parent ? newNode.parent->parent->id : 0, newNode.parent->id, newNode.id);
				
				dtTileNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;
				
				if (!((actualNode->flags & dtTileNode::OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & dtTileNode::CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= ~dtTileNode::CLOSED;
					actualNode->parent = newNode.parent;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;
					
					if (actualNode->flags & dtTileNode::OPEN)
					{
						m_openList->modify(actualNode);
					}
					else
					{
						actualNode->flags = dtTileNode::OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
	}
	
	// Calc hit normal.
	vsub(hitNormal, centerPos, hitPos);
	vnormalize(hitNormal);
	
	return sqrtf(radiusSqr);
}

