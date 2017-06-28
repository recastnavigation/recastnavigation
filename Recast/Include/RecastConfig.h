#ifndef RECASTCONFIG_H
#define RECASTCONFIG_H

/// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
/// If the mesh is appearing below the actual terrain, this parameter may
/// need to be increased.
static const int RC_SPAN_HEIGHT_BITS = 16;

#endif // RECASTCONFIG_H