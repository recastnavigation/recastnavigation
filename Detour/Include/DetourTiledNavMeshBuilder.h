#ifndef DETOURTILEDNAVMESHBUILDER_H
#define DETOURTILEDNAVMESHBUILDER_H

bool dtCreateNavMeshTileData(const unsigned short* verts, const int nverts,
							 const unsigned short* polys, const int npolys, const int nvp,
							 const float* bmin, const float* bmax, float cs, float ch, int tileSize, int walkableClimb,
							 unsigned char** outData, int* outDataSize);

#endif // DETOURTILEDNAVMESHBUILDER_H