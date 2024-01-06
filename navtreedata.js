/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Recast Navigation", "index.html", [
    [ "Introduction", "md_Docs__1_Introducation.html", [
      [ "🚀 Features", "index.html#autotoc_md0", null ],
      [ "⚡ Getting Started", "index.html#autotoc_md1", null ],
      [ "⚙ How it Works", "index.html#autotoc_md2", null ],
      [ "📚 Documentation & Links", "index.html#autotoc_md3", null ],
      [ "❤ Community", "index.html#autotoc_md4", null ],
      [ "⚖ License", "index.html#autotoc_md5", null ],
      [ "What is a Navmesh and how does it work?", "md_Docs__1_Introducation.html#autotoc_md7", [
        [ "Agent Attributes", "md_Docs__1_Introducation.html#autotoc_md8", null ]
      ] ],
      [ "What is a Navmesh not?", "md_Docs__1_Introducation.html#autotoc_md9", null ],
      [ "What is Recast Navigation?", "md_Docs__1_Introducation.html#autotoc_md10", null ],
      [ "High-level overview of the Recast Navmesh-Building Process", "md_Docs__1_Introducation.html#autotoc_md11", null ]
    ] ],
    [ "Building & Integrating", "md_Docs__2_BuildingAndIntegrating.html", [
      [ "Building RecastDemo", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md13", [
        [ "Windows", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md14", null ],
        [ "macOS", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md15", null ],
        [ "Linux", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md16", null ]
      ] ],
      [ "Preprocessor Defines", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md17", null ],
      [ "Running Unit tests", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md18", null ],
      [ "Integration", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md19", [
        [ "Source Integration", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md20", null ],
        [ "Installation through vcpkg", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md21", null ]
      ] ],
      [ "Customizing Allocation Behavior", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md22", null ],
      [ "A Note on DLL exports and C API", "md_Docs__2_BuildingAndIntegrating.html#autotoc_md23", null ]
    ] ],
    [ "FAQ", "md_Docs__3_FAQ.html", [
      [ "Which C++ version and features do Recast use?", "md_Docs__3_FAQ.html#autotoc_md25", null ],
      [ "Why doesn't Recast use STL/Exceptions/RTTI/C++11/my favorite C++ feature?", "md_Docs__3_FAQ.html#autotoc_md26", null ],
      [ "How do I use Recast to build a navmesh?", "md_Docs__3_FAQ.html#autotoc_md27", null ],
      [ "How do Recast and Detour handle memory allocations?", "md_Docs__3_FAQ.html#autotoc_md28", null ],
      [ "Does Recast do any logging?", "md_Docs__3_FAQ.html#autotoc_md29", null ],
      [ "What are the dependencies for RecastDemo?", "md_Docs__3_FAQ.html#autotoc_md30", null ]
    ] ],
    [ "Development Roadmap", "md_Docs__99_Roadmap.html", [
      [ "Short Term", "md_Docs__99_Roadmap.html#autotoc_md32", [
        [ "Documentation & Web Presence (WIP)", "md_Docs__99_Roadmap.html#autotoc_md33", null ],
        [ "More explicit variable names (WIP)", "md_Docs__99_Roadmap.html#autotoc_md34", null ],
        [ "Opt-in config value validation system", "md_Docs__99_Roadmap.html#autotoc_md35", null ]
      ] ],
      [ "Medium Term", "md_Docs__99_Roadmap.html#autotoc_md36", [
        [ "STB-Style Single-Header Release Packaging", "md_Docs__99_Roadmap.html#autotoc_md37", null ],
        [ "Ensure there's a good threading story", "md_Docs__99_Roadmap.html#autotoc_md38", null ],
        [ "More Tests", "md_Docs__99_Roadmap.html#autotoc_md39", null ],
        [ "More POD structs for clarity in internals (WIP)", "md_Docs__99_Roadmap.html#autotoc_md40", null ],
        [ "Revisit structural organization", "md_Docs__99_Roadmap.html#autotoc_md41", null ]
      ] ],
      [ "Longer-Term", "md_Docs__99_Roadmap.html#autotoc_md42", [
        [ "Higher-Level APIs", "md_Docs__99_Roadmap.html#autotoc_md43", null ],
        [ "C API", "md_Docs__99_Roadmap.html#autotoc_md44", null ]
      ] ],
      [ "New Recast/Detour Functionality", "md_Docs__99_Roadmap.html#autotoc_md45", [
        [ "Nav Volumes & 3D Navigation", "md_Docs__99_Roadmap.html#autotoc_md46", null ],
        [ "Climbing Markup & Navigation", "md_Docs__99_Roadmap.html#autotoc_md47", null ],
        [ "Tooling", "md_Docs__99_Roadmap.html#autotoc_md48", null ],
        [ "More spatial querying abilities", "md_Docs__99_Roadmap.html#autotoc_md49", null ],
        [ "Auto-markup system", "md_Docs__99_Roadmap.html#autotoc_md50", null ],
        [ "Formations, Group behaviors", "md_Docs__99_Roadmap.html#autotoc_md51", null ],
        [ "Vehicle Navigation & Movement", "md_Docs__99_Roadmap.html#autotoc_md52", null ],
        [ "Crowd Simulation and Flowfield movement systems", "md_Docs__99_Roadmap.html#autotoc_md53", null ]
      ] ]
    ] ],
    [ "Changelog", "md_CHANGELOG.html", [
      [ "<a href=\"https://github.com/recastnavigation/recastnavigation/compare/1.6.0...HEAD\"", "md_CHANGELOG.html#autotoc_md55", null ],
      [ "<a href=\"https://github.com/recastnavigation/recastnavigation/compare/1.5.1...1.6.0\">1.6.0</a> - 2023-05-21", "md_CHANGELOG.html#autotoc_md56", [
        [ "Added", "md_CHANGELOG.html#autotoc_md57", null ],
        [ "Fixed", "md_CHANGELOG.html#autotoc_md58", null ],
        [ "Changed", "md_CHANGELOG.html#autotoc_md59", null ],
        [ "Removed", "md_CHANGELOG.html#autotoc_md60", null ]
      ] ],
      [ "<a href=\"https://github.com/recastnavigation/recastnavigation/compare/1.5.0...1.5.1\">1.5.1</a> - 2016-02-22", "md_CHANGELOG.html#autotoc_md61", null ],
      [ "1.5.0 - 2016-01-24", "md_CHANGELOG.html#autotoc_md62", null ],
      [ "1.4.0 - 2009-08-24", "md_CHANGELOG.html#autotoc_md63", null ],
      [ "1.3.1 - 2009-07-24", "md_CHANGELOG.html#autotoc_md64", null ],
      [ "1.3.1 - 2009-07-14", "md_CHANGELOG.html#autotoc_md65", null ],
      [ "1.2.0 - 2009-06-17", "md_CHANGELOG.html#autotoc_md66", null ],
      [ "1.1.0 - 2009-04-11", "md_CHANGELOG.html#autotoc_md67", null ],
      [ "1.0.0 - 2009-03-29", "md_CHANGELOG.html#autotoc_md68", null ]
    ] ],
    [ "Code of Conduct", "md_CODE_OF_CONDUCT.html", [
      [ "Our Pledge", "md_CODE_OF_CONDUCT.html#autotoc_md70", null ],
      [ "Our Standards", "md_CODE_OF_CONDUCT.html#autotoc_md71", null ],
      [ "Enforcement Responsibilities", "md_CODE_OF_CONDUCT.html#autotoc_md72", null ],
      [ "Scope", "md_CODE_OF_CONDUCT.html#autotoc_md73", null ],
      [ "Enforcement", "md_CODE_OF_CONDUCT.html#autotoc_md74", null ],
      [ "Enforcement Guidelines", "md_CODE_OF_CONDUCT.html#autotoc_md75", [
        [ "1. Correction", "md_CODE_OF_CONDUCT.html#autotoc_md76", null ],
        [ "2. Warning", "md_CODE_OF_CONDUCT.html#autotoc_md77", null ],
        [ "3. Temporary Ban", "md_CODE_OF_CONDUCT.html#autotoc_md78", null ],
        [ "4. Permanent Ban", "md_CODE_OF_CONDUCT.html#autotoc_md79", null ]
      ] ],
      [ "Attribution", "md_CODE_OF_CONDUCT.html#autotoc_md80", null ]
    ] ],
    [ "Contribution Guidelines", "md_CONTRIBUTING.html", [
      [ "Code of Conduct", "md_CONTRIBUTING.html#autotoc_md82", null ],
      [ "Have a Question or Problem?", "md_CONTRIBUTING.html#autotoc_md83", null ],
      [ "Want a New Feature?", "md_CONTRIBUTING.html#autotoc_md84", null ],
      [ "Found a Bug?", "md_CONTRIBUTING.html#autotoc_md85", [
        [ "Submitting an Issue", "md_CONTRIBUTING.html#autotoc_md86", null ],
        [ "Submitting a Pull Request", "md_CONTRIBUTING.html#autotoc_md87", null ],
        [ "Commit Message Format", "md_CONTRIBUTING.html#autotoc_md88", null ]
      ] ]
    ] ],
    [ "Modules", "modules.html", "modules" ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Typedefs", "functions_type.html", null ],
        [ "Enumerations", "functions_enum.html", null ],
        [ "Enumerator", "functions_eval.html", null ],
        [ "Related Functions", "functions_rela.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Variables", "globals_vars.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"ChunkyTriMesh_8cpp.html",
"DetourDebugDraw_8h_source.html",
"Filelist_8h.html#ae20514067f3272c41e207bed25e530d8",
"RecastRegion_8cpp.html#a25912e8836b3489f54fecd65d850beff",
"classInputGeom.html#a18f14e5605e256a5793515a66f77887d",
"classSample__TempObstacles.html#ac778775311b1bbadbc0e03b0df4f177c",
"classdtNodeQueue.html#a836fbfbac11e06eeadd76919a9bba943",
"functions_c.html",
"group__recast.html#ga6ef022fcf6dabdb0437b979304c0defe",
"md_CONTRIBUTING.html",
"structdtCrowdAgent.html#a118c511ca6807b818da75bcd320fb21f",
"structdtTileCacheLayer.html#aa81a7e641ccfb8e00f0072efae4d74fd",
"structrcPolyMesh.html"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';