#ifndef __AOI_IMPL_ALLOC_H__
#define __AOI_IMPL_ALLOC_H__

#include <functional>
#include <cassert>
#include <cstdlib>

namespace aoi
{
    namespace impl
    {
        template<typename T>
        class MemBase
        {
        public:
            MemBase() {}
            virtual ~MemBase() {}

            virtual void* _alloc(size_t size) = 0;
            virtual void _free(void* ptr) = 0;

            template<typename ... Args>
            inline T* New(Args... args)
            {
                void *ptr = _alloc(sizeof(T));
                return new (ptr)T(args...);
            }
            inline void Delete(T* ptr)
            {
                ptr->~T();
                _free(ptr);
            }
        };

        template<typename T, unsigned BlockSize = 4096>
        class Blocks : public MemBase<T>
        {
        public:
            using TMallocFunc = std::function<void*(size_t)>;
            using TFreeFunc = std::function<void(void*)>;

            Blocks(const TMallocFunc& f1, const TFreeFunc& f2)
                : mBlocks(nullptr)
                , mHead(nullptr)
                , mMalloc(f1)
                , mFree(f2)
            {
            }

            ~Blocks()
            {
                while (mBlocks)
                {
                    Item* next = mBlocks->next;
                    mFree(mBlocks);
                    mBlocks = next;
                }
            }

            inline void* _alloc(size_t size)
            {
                if (!mHead)
                {
                    newBlock();
                }
                void* ptr = mHead;
                mHead = mHead->next;
                return ptr;
            }

            inline void _free(void* ptr)
            {
                Item* p = (Item*)ptr;
                p->next = mHead;
                mHead = p;
            }

        private:
            void newBlock()
            {
                assert(!mHead);
                Item* ptr = (Item*)mMalloc(BlockSize);
                if (mBlocks)
                {
                    ptr->next = mBlocks;
                    mBlocks = ptr;
                }
                else
                {
                    mBlocks = ptr;
                    mBlocks->next = 0;
                }
                ptr = ptr + 1;

#define PTR(N) ((Item*)((char*)ptr + (N) * sizeof(T)))
                mHead = PTR(0);
                size_t lstIndex = (BlockSize - sizeof(Item)) / sizeof(T) - 1;
                for (size_t i = 0; i < lstIndex; i++)
                {
                    PTR(i)->next = PTR(i + 1);
                }
                PTR(lstIndex)->next = nullptr;
#undef PTR
            }

            struct Item
            {
                Item* next;
            };
            Item* mBlocks;
            Item* mHead;
            TMallocFunc mMalloc;
            TFreeFunc mFree;
        };


        template<typename T, unsigned BlockSize = 4096>
        class Mem : public Blocks<T, BlockSize>
        {
        public:
            Mem() : Blocks<T, BlockSize>(malloc, free)
            {
            }

            ~Mem()
            {
            }
        };

        template<typename T, unsigned BlockSize = 4096, unsigned Alignment = 4>
        class AlignedMem : public Blocks<T, BlockSize>
        {
        public:
#ifdef _MSC_VER
            AlignedMem() : Blocks<T, BlockSize>(std::bind(_aligned_malloc, std::placeholders::_1, Alignment), _aligned_free)
#else
            AlignedMem() : Blocks<T, BlockSize>(std::bind(aligned_alloc, Alignment, std::placeholders::_1), free)
#endif
            {
            }

            ~AlignedMem()
            {
            }
        };
    }
}

#endif
