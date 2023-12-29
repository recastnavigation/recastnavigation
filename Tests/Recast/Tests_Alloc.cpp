#include <stdio.h>
#include <string.h>

#include "catch2/catch_all.hpp"

#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Used to verify that rcVector constructs/destroys objects correctly.
struct Incrementor {
	static int constructions;
	static int destructions;
	static int copies;
	Incrementor() { constructions++; }
	~Incrementor() { destructions++; }
	Incrementor(const Incrementor&) { copies++; }
	Incrementor& operator=(const Incrementor&); // Deleted assignment.

	static void Reset() {
		constructions = 0;
		destructions = 0;
		copies = 0;
	}
};

int Incrementor::constructions = 0;
int Incrementor::destructions = 0;
int Incrementor::copies = 0;

const int kMaxAllocSize = 1024;
const unsigned char kClearValue = 0xff;

/// Simple alloc/free that clears the memory on free..
void* AllocAndInit(size_t size, rcAllocHint) {
	rcAssert(kMaxAllocSize >= size);
	return memset(malloc(kMaxAllocSize), 0, kMaxAllocSize);
}

void FreeAndClear(void* mem) {
	if (mem) {
	  memset(mem, kClearValue, kMaxAllocSize);
	}
	free(mem);
}

// Verifies that memory has been initialized by AllocAndInit, and not cleared by FreeAndClear.
struct Copier {
	const static int kAlive;
	const static int kDead;
	Copier() : value(kAlive) {}

	// checks that the source of the copy is valid.
	Copier(const Copier& other) : value(kAlive) {
		other.Verify();
	}
	Copier& operator=(const Copier&);

	// Marks the value as dead.
	~Copier() { value = kDead; }
	void Verify() const {
		REQUIRE(value == kAlive);
	}
	volatile int value;
};

const int Copier::kAlive = 0x1f;
const int Copier::kDead = 0xde;

struct NotDefaultConstructible {
	NotDefaultConstructible(int) {}
};

TEST_CASE("rcVector", "[recast, alloc]")
{
	SECTION("Vector basics.")
	{
		rcTempVector<int> vec;
		REQUIRE(vec.size() == 0);
		vec.push_back(10);
		vec.push_back(12);
		REQUIRE(vec.size() == 2);
		REQUIRE(vec.capacity() >= 2);
		REQUIRE(vec[0] == 10);
		REQUIRE(vec[1] == 12);
		vec.pop_back();
		REQUIRE(vec.size() == 1);
		REQUIRE(vec[0] == 10);
		vec.pop_back();
		REQUIRE(vec.size() == 0);
		vec.resize(100, 5);
		REQUIRE(vec.size() == 100);
		for (int i = 0; i < 100; i++) {
			REQUIRE(vec[i] == 5);
			vec[i] = i;
		}
		for (int i = 0; i < 100; i++) {
			REQUIRE(vec[i] == i);
		}
	}

	SECTION("Constructors/Destructors")
	{
		Incrementor::Reset();
		rcTempVector<Incrementor> vec;
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);
		vec.push_back(Incrementor());
		// push_back() may create and copy objects internally.
		REQUIRE(Incrementor::constructions == 1);
		REQUIRE(Incrementor::destructions >= 1);
		// REQUIRE(Incrementor::copies >= 2);

		vec.clear();
		Incrementor::Reset();
		vec.resize(100);
		// Initialized with default instance. Temporaries may be constructed, then destroyed.
		REQUIRE(Incrementor::constructions == 100);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);

		Incrementor::Reset();
		for (int i = 0; i < 100; i++) {
			REQUIRE(Incrementor::destructions == i);
			vec.pop_back();
		}
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 100);
		REQUIRE(Incrementor::copies == 0);

		vec.resize(100);
		Incrementor::Reset();
		vec.clear();
		// One temp object is constructed for the default argument of resize().
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 100);
		REQUIRE(Incrementor::copies == 0);

		Incrementor::Reset();
		vec.resize(100, Incrementor());
		REQUIRE(Incrementor::constructions == 1);
		REQUIRE(Incrementor::destructions == 1);
		REQUIRE(Incrementor::copies == 100);
	}

	SECTION("Copying Contents")
	{

		// veriyf event counts after doubling size -- should require a lot of copying and destroying.
		rcTempVector<Incrementor> vec;
		Incrementor::Reset();
		vec.resize(100);
		REQUIRE(Incrementor::constructions == 100);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);
		Incrementor::Reset();
		vec.resize(200);
		REQUIRE(vec.size() == vec.capacity());
		REQUIRE(Incrementor::constructions == 100);  // Construc new elements.
		REQUIRE(Incrementor::destructions == 100);  // Destroy old contents.
		REQUIRE(Incrementor::copies == 100);  // Copy old elements into new array.
	}

	SECTION("Swap")
	{
		rcTempVector<int> a(10, 0xa);
		rcTempVector<int> b;

		int* a_data = a.data();
		int* b_data = b.data();

		a.swap(b);
		REQUIRE(a.size() == 0);
		REQUIRE(b.size() == 10);
		REQUIRE(b[0] == 0xa);
		REQUIRE(b[9] == 0xa);
		REQUIRE(a.data() == b_data);
		REQUIRE(b.data() == a_data);
	}

	SECTION("Overlapping init")
	{
		rcAllocSetCustom(&AllocAndInit, &FreeAndClear);
		rcTempVector<Copier> vec;
		// Force a realloc during push_back().
		vec.resize(64);
		REQUIRE(vec.capacity() == vec.size());
		REQUIRE(vec.capacity() > 0);
		REQUIRE(vec.size() == vec.capacity());

		// Don't crash.
		vec.push_back(vec[0]);
		rcAllocSetCustom(NULL, NULL);
	}

	SECTION("Vector Destructor")
	{
		{
			rcTempVector<Incrementor> vec;
			vec.resize(10);
			Incrementor::Reset();
		}
		REQUIRE(Incrementor::destructions == 10);
	}

	SECTION("Assign")
	{
		rcTempVector<int> a(10, 0xa);
		a.assign(5, 0xb);
		REQUIRE(a.size() == 5);
		REQUIRE(a[0] == 0xb);
		REQUIRE(a[4] == 0xb);
		a.assign(15, 0xc);
		REQUIRE(a.size() == 15);
		REQUIRE(a[0] == 0xc);
		REQUIRE(a[14] == 0xc);

		rcTempVector<int> b;
		b.assign(a.data(), a.data() + a.size());
		REQUIRE(b.size() == a.size());
		REQUIRE(b[0] == a[0]);
	}

	SECTION("Copy")
	{
		rcTempVector<int> a(10, 0xa);
		rcTempVector<int> b(a);
		REQUIRE(a.size() == 10);
		REQUIRE(a.size() == b.size());
		REQUIRE(a[0] == b[0]);
		REQUIRE(a.data() != b.data());
		rcTempVector<int> c(a.data(), a.data() + a.size());
		REQUIRE(c.size() == a.size());
		REQUIRE(c[0] == a[0]);

		rcTempVector<Incrementor> d(10);
		Incrementor::Reset();
		rcTempVector<Incrementor> e(d);
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 10);

		Incrementor::Reset();
		rcTempVector<Incrementor> f(d.data(), d.data() + d.size());
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 10);
	}

	SECTION("Type Requirements")
	{
		// This section verifies that we don't enforce unnecessary
		// requirements on the types we hold.

		// Implementing clear as resize(0) will cause this to fail
		// as resize(0) requires T to be default constructible.
		rcTempVector<NotDefaultConstructible> v;
		v.clear();
	}
}
