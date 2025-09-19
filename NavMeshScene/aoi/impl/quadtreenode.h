#ifndef __AOI_IMPL_QUADTREENODE_H__
#define __AOI_IMPL_QUADTREENODE_H__

#include "alloc.h"
#include "../rect.h"
#include <memory>

namespace aoi
{
    namespace impl
    {
        enum ENodeType
        {
            NodeTypeNormal = 0,  // Non-leaf node
            NodeTypeLeaf = 1,    // Leaf node
        };

        const unsigned ChildrenNum = 4;

        template<typename TItem, unsigned NodeCapacity, unsigned LevelLimit>
        class QuadTreeNode
        {
        public:
            using TNode = QuadTreeNode<TItem, NodeCapacity, LevelLimit>;

            QuadTreeNode(unsigned level, MemBase<TNode>* alloc, ENodeType type, QuadTreeNode* parent, const Rect& bounds);
            ~QuadTreeNode();

            bool Insert(TItem* item);
            bool Remove(TItem* item);
            void Query(const Rect& area, TItem*& head, TItem*& tail);
            unsigned GetItemCount();

        public:
            unsigned mLevel;                         // The level at which the current node is located
            Rect mBounds;                            // Node border range
            QuadTreeNode* mParent;                   // Parent node
            ENodeType mNodeType;                     // Node type
            QuadTreeNode* mChildrens[ChildrenNum];   // Child node
            unsigned mItemCount;                     // Number of items on the leaf node
            TItem* mItems;                           // Items on the leaf node

        private:
            MemBase<TNode>* mAlloc;                  // Node allocator
            void split();
            void tryMerge();
        };
    }
}

#include "quadtreenode_impl.h"

#endif
