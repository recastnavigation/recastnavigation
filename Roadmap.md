# Recast Development Roadmap

This document describes the current development roadmap for Recast and Detour.  It is initially focused on maintaining and polishing the current functionality in a way that makes it more appealing and easier to use.  There are a number of large areas of new functionality that are appealing as well.  

If you're excited about contributing to Recast or want to understand what its future looks like, this roadmap is a great place to start.

## Short Term

### Documentation & Web Presence
-   **Project website** (GitHub pages). A home for docs, info, tutorials, etc. that's easy to find and navigate. There's stuff like the wiki system in GitHub that can serve this purpose, but it's not the greatest.
-   **Hosted API Reference**: We have extensive doxygen docs that we should also host on github pages.  Ideally this would be implemented as a job for the CI process.
-   **High-level design/overview**. Basically taking a lot of the "how does Recast work?" docs we have (and stuff on Mikko's blog) and surfacing them in a place that's easier to find.  e.g. [this information](http://digestingduck.blogspot.com/2010/02/slides-from-past.html) should be integrated to the documentation.
-   **FAQ** to include answers to common questions like "can I use Recast on a spherical world?" etc. The small group of questions that come up often.
-   **Expand on configuration parameter docstrings**. Expand on the docstrings for the fields in `rcConfig`.
-   **Projects using recast page**. A place to show off integrations of Recast into different games and engines. Having this visible will help give people confidence in the quality of Recast.

### More explicit variable names
There's quite a few variables that are single-letter or very condensed acronyms. Stuff like `u`, `v`, `cs`, `nnei` etc. These are certainly easier to type than more descriptive names, and can allow for some nice code formatting, but the readability gains of less condensed names outweigh that.  These should be updated to be more easily maintainable and less of a hurdle for newcomers.

### Opt-in config value validation system
There's a number of documentation-defined valid value ranges, as well as some suggested value ranges for config values like those in rcConfig.  Recast should have an _opt-in_ system for validating value ranges and value relationships.  Some implementations [like Godot's](https://github.com/godotengine/godot/blob/c7ceb94e372216b1b033d7c2ac26d5b7545c4dac/modules/navigation/navigation_mesh_generator.cpp#L545-L568) include similar configuration value checks already.  This would help alert users to potential problems arising from misconfiguration.  Recast shouldn't restrict people from going outside the recommended bounds, but for the majority of users values that are out of bounds are likely a mistake and Recast should at least raise the concern when prompted.

## Medium Term

### STB-Style Single-Header Release Packaging
For ease of integration. This should probably be implemented as a build packaging script, similar to the release packaging process for the Catch unit testing library.

### Ensure there's a good threading story
Currently Recast provides no specific threading support.  Many aspects of navmesh generation (especialy with tile generation) can be easily multithreaded, as can parts of detour (threaded pathfinding, etc).  This likely requires a good example integration to work with - it probably shouldn't be designed in isolation.

### More Tests
This is a good place to start when making any changes to the internals or fixing bugs. It's important to have a way to validate that bugfixes or enhancements are not adversely affecting the base Recast functionality. Lots of parts of the code are fairly simple to test, but some would require some extra effort with mocking data or similar.

### More POD structs for clarity in internals
Small structs like a simple 3-float POD vector would go a long way to help clarify a lot of the internals. Any changes here shouldn't impact performance or functionality. They should just be simple POD structs that get optimized away but make the code a bit clearer to read and work with.

### Revisit structural organization
There some number of generally low-level geometry functions that are implemented both in Recast and in Detour. We should probably take stock of the general library layout and determine if there's enough of a benefit to slightly restructuring things to better share core functionality. There may not be enough code like this to be worth it, so it might not be a net win.  Organization and re-use benefits should vastly outweighing the cost of defining a shared dependency to make this worthwhile.  There's likely just minimal gains here since things are already laid out logically, but it's worth looking into.

## Longer-Term

### Higher-Level APIs
The current API level is excellent for providing maximum control over the entire navmesh building process. For many people, integrating Recast involves copying over all the code from the tiled mesh demo with minimal tweaks. Recast can better serve these people, and hopefully boost adoption by providing some higher-level API calls.

### C API
This would help tremendously with adoption. There are a number of integrations of Recast to other languages right now, and a number of them are using their own C API wrappers around the existing Recast API. Providing a first-party C API would help normalize the different integrations of Recast out there, as well as improve the integration process for people with engines written in C or other languages with C interop.

There's enough cases where we pass around object pointers in API calls that may cause this to be a bit more work than it seems.

# Roadmap for New Recast/Detour Functionality

### Nav Volumes & 3D Navigation
Obviously a big feature that would enable flying agents. Probably not terribly difficult to integrate due to the voxelization process, but it's probably a lot of detailed geometric processes and challenges. We'd at least need to retain unwalkable polygons longer in the process (they get thrown away pretty early) so we should ensure it doesn't have any adversarial affects to the perf or capabilities of the normal navmesh process.  3d navigation should be a separate set of APIs, as the considerations and use cases are quite different than normal navmesh navigation.

### Climbing Markup & Navigation
This is a common problem with many modern games.  There's a number of ways to solve this, so we should make sure to consult with people who are actively using Recast and building games with climbing to understand their needs. 

### Tooling
Recast Demo already has a number of sample tools to explore and test the generated navmesh data. There's a number of middleware providers that tout workflow features and tooling for generating and manipulating navmesh data. RecastDemo could be expanded (and maybe re-named) to be a more fully fleshed out standalone tool, rather than be mostly focused as an integration demo and feature showcase. There's almost certainly a ton of custom data and markup most people will want to add to the generated navmeshes, so there should be a reasonable way for people to integrate their customizations and markup with any tooling Recast provide.

### More spatial querying abilities
There's already a number of spatial query functions, and fleshing out that set some more is likely quite easy and useful.

### Auto-markup system
This is something recast already gives you access to add yourself, thanks to its low-level API and how each step in the process is explicit.  It is worthwhile to consider what kind of help other middleware systems provide in this process. For example, are other navigation middleware solutions providing a structured way to define spatial queries that result in nav markup? Some easy examples of useful features here might be automatic jump point or cover annotations. It would be good to talk to people in the AAA FPS space to see what markup types are most useful to them.

### Formations, Group behaviors
Extending DetourCrowd to support formations, group navigation, group behaviors and similar beyond what's it's already capable of.

### Vehicle Navigation & Movement
A feature set centered on pathfinding and movement planning for characters with kinematic movement constraints.

### Crowd Simulation and Flowfield movement systems
Movement systems for large crowds that leverage flowfields or other highly-scalable navigation and movement approaches.