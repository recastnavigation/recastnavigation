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

#pragma once
#include "DetourNavMesh.h"
#include <cstdint>

enum dtNodeFlags
{
    DT_NODE_OPEN = 0x01,
    DT_NODE_CLOSED = 0x02,
    DT_NODE_PARENT_DETACHED = 0x04 // parent of the node is not adjacent. Found using raycast.
};

typedef uint16_t dtNodeIndex;
static constexpr dtNodeIndex DT_NULL_IDX = static_cast<dtNodeIndex>(~0);

static constexpr int DT_NODE_PARENT_BITS = 24;
static constexpr int DT_NODE_STATE_BITS = 2;

struct dtNode
{
    float pos[3]; ///< Position of the node.
    float cost; ///< Cost from previous node to current node.
    float total; ///< Cost up to the node.
    uint32_t pidx : DT_NODE_PARENT_BITS; ///< Index to parent node.
    uint32_t state : DT_NODE_STATE_BITS;
    ///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
    uint32_t flags : 3; ///< Node flags. A combination of dtNodeFlags.
    dtPolyRef id; ///< Polygon ref the node corresponds to.
};

static constexpr int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;
// number of extra states per node. See dtNode::state

class dtNodePool
{
public:
    dtNodePool(int maxNodes, int hashSize);
    ~dtNodePool();
    void clear();

    // Get a dtNode by ref and extra state information. If there is none then - allocate
    // There can be more than one node for the same polyRef but with different extra state information
    dtNode* getNode(dtPolyRef id, uint8_t state = 0);
    dtNode* findNode(dtPolyRef id, uint8_t state) const;
    uint32_t findNodes(dtPolyRef id, dtNode** nodes, int maxNodes) const;

    uint32_t getNodeIdx(const dtNode* node) const
    {
        if (!node) return 0;
        return static_cast<uint32_t>(node - m_nodes) + 1;
    }

    dtNode* getNodeAtIdx(const uint32_t idx)
    {
        if (!idx) return nullptr;
        return &m_nodes[idx - 1];
    }

    const dtNode* getNodeAtIdx(const uint32_t idx) const
    {
        if (!idx) return nullptr;
        return &m_nodes[idx - 1];
    }

    int getMemUsed() const
    {
        return sizeof(*this) +
            sizeof(dtNode) * m_maxNodes +
            sizeof(dtNodeIndex) * m_maxNodes +
            sizeof(dtNodeIndex) * m_hashSize;
    }

    int getMaxNodes() const { return m_maxNodes; }

    int getHashSize() const { return m_hashSize; }
    dtNodeIndex getFirst(const int bucket) const { return m_first[bucket]; }
    dtNodeIndex getNext(const int i) const { return m_next[i]; }
    int getNodeCount() const { return m_nodeCount; }

    // Explicitly disabled copy constructor and copy assignment operator.
    dtNodePool(const dtNodePool&) = delete;
    dtNodePool& operator=(const dtNodePool&) = delete;
private:

    dtNode* m_nodes;
    dtNodeIndex* m_first;
    dtNodeIndex* m_next;
    const int m_maxNodes;
    const int m_hashSize;
    int m_nodeCount;
};

class dtNodeQueue
{
public:
    explicit dtNodeQueue(int n);
    ~dtNodeQueue();

    void clear() { m_size = 0; }

    dtNode* top() const { return m_heap[0]; }

    dtNode* pop()
    {
        dtNode* result = m_heap[0];
        m_size--;
        trickleDown(0, m_heap[m_size]);
        return result;
    }

    void push(dtNode* node)
    {
        m_size++;
        bubbleUp(m_size - 1, node);
    }

    void modify(dtNode* node) const {
        for (int i = 0; i < m_size; ++i)
        {
            if (m_heap[i] == node)
            {
                bubbleUp(i, node);
                return;
            }
        }
    }

    bool empty() const { return m_size == 0; }

    int getMemUsed() const
    {
        return sizeof(*this) +
            sizeof(dtNode*) * (m_capacity + 1);
    }

    int getCapacity() const { return m_capacity; }

    // Explicitly disabled copy constructor and copy assignment operator.
    dtNodeQueue(const dtNodeQueue&) = delete;
    dtNodeQueue& operator=(const dtNodeQueue&) = delete;

private:
    void bubbleUp(int i, dtNode* node) const;
    void trickleDown(int i, dtNode* node) const;

    dtNode** m_heap;
    const int m_capacity;
    int m_size;
};
