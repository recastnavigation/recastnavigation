# FAQ

## Which C++ version and features do Recast use?

All code in Recast and Detour strictly adheres to the following:

* C++98
* no STL
* no exceptions
* no RTTI
* minimal inheritance
* minimal templates

RecastDemo is a bit looser with these requirements, as it's only meant to showcase Recast usage and functionality, not be part of a shipped product.

## What coordinate system and triangle winding order does Recast use?

Recast expects clockwise-winding triangles and uses a right-handed, Y-up coordinate system.

## Why doesn't Recast use STL/Exceptions/RTTI/C++11/my favorite C++ feature?

Recast has always strived to maximize its ease of integration into an existing codebase, its portability to unique platforms, and its runtime performance.  Recast was forged in the fires of game development, and as such follows the cultural norms of the games industry.

For example, some platforms have limited C++ compilers and STL implementations, so avoiding newer C++ features and the STL entirely helps tremendously when working in those environments.  Additionally, exceptions and RTTI require a non-trivial runtime overhead to support.  This is in conflict with Recast's goal of maximum performance.

## How do I use Recast to build a navmesh?

The process is thoroughly outlined and documented in the RecastDemo project.  `Sample_SoloMesh.cpp` is a good introduction to the general process of building a navmesh.  It builds a single, unified navmesh and is the simpler and more limited of the two examples.  `Sample_TileMesh.cpp` builds a tiled navmesh that supports all of Recast and Detour's features around dynamic obstacles and runtime re-meshing.

## How do Recast and Detour handle memory allocations?

Recast and Detour strive to avoid heap allocations whenver possible.  When heap allocations are necessary, they are routed through centralized allocation and de-allocation functions.  These centralized functions allow you to optionally override them with custom allocator implementations, but just use `malloc` and `free` by default.  Check out `RecastAlloc.h` and `DetourAlloc.h` for more.

## Does Recast do any logging?

Yes and no: recast provides a `log(...)` function through `rcContext`. There are currently 3 levels: progress, warning, and error. If logging is enabled, the context calls the doLog() virtual function which is empty by default.

## What are the dependencies for RecastDemo?

* SDL 2
* OpenGL
* GLU
