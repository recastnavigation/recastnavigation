#include <stdio.h>
#include <string.h>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include <vector>

// TODO: Implement benchmarking for platforms other than posix.
#ifdef __unix__
#include <unistd.h>
#ifdef _POSIX_TIMERS
#include <time.h>
#include <stdint.h>

int64_t NowNanos() {
	struct timespec tp;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tp);
	return tp.tv_nsec + 1000000000LL * tp.tv_sec;
}

#define BM(name, iterations) \
	struct BM_ ## name { \
		static void Run() { \
			int64_t begin_time = NowNanos(); \
			for (int i = 0 ; i < iterations; i++) { \
				Body(); \
			} \
			int64_t nanos = NowNanos() - begin_time; \
			printf("BM_%-35s %ld iterations in %10ld nanos: %10.2f nanos/it\n", #name ":", (int64_t)iterations, nanos, double(nanos) / iterations); \
		} \
		static void Body(); \
	}; \
	TEST_CASE(#name) { \
		BM_ ## name::Run(); \
	} \
	void BM_ ## name::Body()

const int64_t kNumLoops = 100;
const int64_t kNumInserts = 100000;

// Prevent compiler from eliding a calculation.
// TODO: Implement for MSVC.
template <typename T>
void DoNotOptimize(T* v) {
	asm volatile ("" : "+r" (v));
}

BM(FlatArray_Push, kNumLoops)
{
	int cap = 64;
	int* v = (int*)rcAlloc(cap * sizeof(int), RC_ALLOC_TEMP);
	for (int j = 0; j < kNumInserts; j++) {
		if (j == cap) {
			cap *= 2;
			int* tmp  = (int*)rcAlloc(sizeof(int) * cap, RC_ALLOC_TEMP);
			memcpy(tmp, v, j * sizeof(int));
			rcFree(v);
			v = tmp;
		}
		v[j] = 2;
	}

	DoNotOptimize(v);
	rcFree(v);
}
BM(FlatArray_Fill, kNumLoops)
{
	int* v = (int*)rcAlloc(sizeof(int) * kNumInserts, RC_ALLOC_TEMP);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}

	DoNotOptimize(v);
	rcFree(v);
}
BM(FlatArray_Memset, kNumLoops)
{
	int* v = (int*)rcAlloc(sizeof(int) * kNumInserts, RC_ALLOC_TEMP);
	memset(v, 0, kNumInserts * sizeof(int));

	DoNotOptimize(v);
	rcFree(v);
}

BM(rcVector_Push, kNumLoops)
{
	rcTempVector<int> v;
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(rcVector_PushPreallocated, kNumLoops)
{
	rcTempVector<int> v;
	v.reserve(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(rcVector_Assign, kNumLoops)
{
	rcTempVector<int> v;
	v.assign(kNumInserts, 2);
	DoNotOptimize(v.data());
}
BM(rcVector_AssignIndices, kNumLoops)
{
	rcTempVector<int> v;
	v.resize(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}
	DoNotOptimize(v.data());
}
BM(rcVector_Resize, kNumLoops)
{
	rcTempVector<int> v;
	v.resize(kNumInserts, 2);
	DoNotOptimize(v.data());
}

BM(stdvector_Push, kNumLoops)
{
	std::vector<int> v;
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(stdvector_PushPreallocated, kNumLoops)
{
	std::vector<int> v;
	v.reserve(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(stdvector_Assign, kNumLoops)
{
	std::vector<int> v;
	v.assign(kNumInserts, 2);
	DoNotOptimize(v.data());
}
BM(stdvector_AssignIndices, kNumLoops)
{
	std::vector<int> v;
	v.resize(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}
	DoNotOptimize(v.data());
}
BM(stdvector_Resize, kNumLoops)
{
	std::vector<int> v;
	v.resize(kNumInserts, 2);
	DoNotOptimize(v.data());
}

#undef BM
#endif  // _POSIX_TIMERS
#endif  // __unix__
