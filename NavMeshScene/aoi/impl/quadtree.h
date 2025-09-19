#ifndef __AOI_IMPL_QUADTREE_H__
#define __AOI_IMPL_QUADTREE_H__

#include "alloc.h"
#include "quadtreenode.h"

namespace aoi
{
    namespace impl
    {
        template<typename TItem, unsigned NodeCapacity, unsigned LevelLimit, typename TAlloc>
        class QuadTree {
        public:
            using TNode = QuadTreeNode<TItem, NodeCapacity, LevelLimit>;

            QuadTree() : mRoot(0, &mAlloc, NodeTypeLeaf, nullptr, Rect()) {}
            QuadTree(const Rect& bounds) : mRoot(0, &mAlloc, NodeTypeLeaf, nullptr, bounds) {}
            ~QuadTree() {}

            bool Insert(TItem* item) { return mRoot.Insert(item); }

            bool Remove(TItem* item)
            {
                TNode* node = (TNode*)item->mNode;
                return node ? node->Remove(item) : false;
            }

            TItem* Query(const Rect& area)
            {
                TItem* head = nullptr;
                TItem* tail = nullptr;
                mRoot.Query(area, head, tail);
                tail ? tail->mQueryNext = nullptr : 0;
                return head;
            }

            inline TItem* Query(TItem* source, float radius)
            {
                Rect area(source->X - radius, source->X + radius, source->Y - radius, source->Y + radius);
                return Query(source, area);
            }

            inline TItem* Query(TItem* source, float halfExtentsX, float halfExtentsY)
            {
                Rect area(source->X - halfExtentsX, source->X + halfExtentsX, source->Y - halfExtentsY, source->Y + halfExtentsY);
                return Query(source, area);
            }

            TItem* Query(TItem* source, const Rect& area)
            {
                TItem* head = nullptr;
                TItem* tail = nullptr;
                TNode* node = (TNode*)source->mNode;
                (node && node->mBounds.Contains(area)) ? node->Query(area, head, tail) : mRoot.Query(area, head, tail);
                tail ? tail->mQueryNext = nullptr : 0;
                return head;
            }

            bool Update(TItem* item)
            {
                TNode* node = (TNode*)item->mNode;
                if (node)
                {
                    if (node->mBounds.Contains(item))
                    {
                        return true;
                    }
                    node->Remove(item);
                }
                return mRoot.Insert(item);
            }

            unsigned GetItemCount() { return mRoot.GetItemCount(); }

            inline Rect& GetBounds() { return mRoot.mBounds; }
            inline void SetBounds(const Rect& rect) { mRoot.mBounds = rect; }

        private:
            TAlloc mAlloc;
            TNode mRoot;
        };
    }
}

#endif
