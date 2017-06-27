#ifndef DETOURCONFIG_H
#define DETOURCONFIG_H

// Undefine the following line to use 64bit polyref.
// Generally not needed, useful for very large worlds.
// Note: tiles built using 32bit refs are not compatible with 64bit refs!
//#define DT_POLYREF64 1

// Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from dtQueryFilter.
// On certain platforms indirect or virtual function call is expensive. The default
// setting is to use non-virtual functions.
//#define DT_VIRTUAL_QUERYFILTER 1

// The data type used to index nodes when doing path queries. If you are doing very large
// queries you might need to change this to unsigned int and pass a higher node count to
// dtNavMeshQuery.
typedef unsigned short dtNodeIndex;

#endif // DETOURCONFIG_H