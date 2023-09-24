# Integration

There are a few ways to integrate Recast and Detour into your project.  Source integration is the most popular and most flexible, and is what the project was designed for from the beginning.

## Source Integration

It is recommended to add the source directories `DebugUtils`, `Detour`, `DetourCrowd`, `DetourTileCache`, and `Recast` directly into your project depending on which parts of the project you need. For example your level building tool could include `DebugUtils`, `Recast`, and `Detour`, and your game runtime could just include `Detour`.

- *Recast*: Core navmesh building system.
- *Detour*: Runtime navmesh interface and query system
- *DetourCrowd*: Runtime movement, obstacle avoidance and crowd sim systems
- *DetourTileCache*: Runtime navmesh dynamic obstacle and re-baking system

## Install through vcpkg

If you are using the [vcpkg](https://github.com/Microsoft/vcpkg/) dependency manager you can alternatively download and install Recast with:

```
vcpkg install recast
```

# DLL and C API exports

Recast does not currently provide a stable C API for use in a DLL or as bindings for another language.  The design of Recast currently relies on some C++ features, so providing a stable API is not possible without a few large changes to Recast.  There are a number of projects that offer unofficial language bindings for Recast, but official support for a C API is currently on our [Roadmap](Roadmap.md).

# Preprocessor defines

`RC_DISABLE_ASSERTS`: Disables assertion macros.  Useful for release builds that need to maximize performance.

`DT_POLYREF64`: Use 64 bit (rather than 32 bit) polygon ID references.  Generally not needed, but sometimes useful for very large worlds.

`DT_VIRTUAL_QUERYFILTER`: Define this if you plan to sub-class `dtQueryFilter`.  Enables the virtual destructor in `dtQueryFilter`