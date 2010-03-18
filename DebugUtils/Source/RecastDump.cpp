//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastDump.h"


bool duDumpPolyMeshToObj(rcPolyMesh& pmesh, const char* filepath)
{
	FILE* fp = fopen(filepath, "w");
	if (!fp)
		return false;
	
	const int nvp = pmesh.nvp;
	const float cs = pmesh.cs;
	const float ch = pmesh.ch;
	const float* orig = pmesh.bmin;
	
	fprintf(fp, "# Recast Navmesh\n");
	fprintf(fp, "o NavMesh\n");

	fprintf(fp, "\n");
	
	for (int i = 0; i < pmesh.nverts; ++i)
	{
		const unsigned short* v = &pmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		fprintf(fp, "v %f %f %f\n", x,y,z);
	}

	fprintf(fp, "\n");

	for (int i = 0; i < pmesh.npolys; ++i)
	{
		const unsigned short* p = &pmesh.polys[i*nvp*2];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			fprintf(fp, "f %d %d %d\n", p[0]+1, p[j-1]+1, p[j]+1); 
		}
	}
	
	fclose(fp);
	
	return true;
}

bool duDumpPolyMeshDetailToObj(rcPolyMeshDetail& dmesh, const char* filepath)
{
	FILE* fp = fopen(filepath, "w");
	if (!fp)
		return false;
	
	fprintf(fp, "# Recast Navmesh\n");
	fprintf(fp, "o NavMesh\n");
	
	fprintf(fp, "\n");

	for (int i = 0; i < dmesh.nverts; ++i)
	{
		const float* v = &dmesh.verts[i*3];
		fprintf(fp, "v %f %f %f\n", v[0],v[1],v[2]);
	}
	
	fprintf(fp, "\n");
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short btris = m[2];
		const unsigned short ntris = m[3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		for (int j = 0; j < ntris; ++j)
		{
			fprintf(fp, "f %d %d %d\n",
					(int)(bverts+tris[j*4+0])+1,
					(int)(bverts+tris[j*4+1])+1,
					(int)(bverts+tris[j*4+2])+1);
		}
	}
	
	fclose(fp);
	
	return true;
}


static const int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static const int CHF_VERSION = 2;

bool duDumpCompactHeightfield(struct rcCompactHeightfield& chf, const char* filepath)
{
	FILE* fp = fopen(filepath, "wb");
	if (!fp)
	{
		printf("duDumpCompactHeightfield: Could not open '%s' for writing.\n", filepath);
		return false;
	}

	fwrite(&CHF_MAGIC, sizeof(CHF_MAGIC), 1, fp);
	fwrite(&CHF_VERSION, sizeof(CHF_VERSION), 1, fp);
	
	fwrite(&chf.width, sizeof(chf.width), 1, fp);
	fwrite(&chf.height, sizeof(chf.height), 1, fp);
	fwrite(&chf.spanCount, sizeof(chf.spanCount), 1, fp);

	fwrite(&chf.walkableHeight, sizeof(chf.walkableHeight), 1, fp);
	fwrite(&chf.walkableClimb, sizeof(chf.walkableClimb), 1, fp);

	fwrite(&chf.maxDistance, sizeof(chf.maxDistance), 1, fp);
	fwrite(&chf.maxRegions, sizeof(chf.maxRegions), 1, fp);

	fwrite(chf.bmin, sizeof(chf.bmin), 1, fp);
	fwrite(chf.bmax, sizeof(chf.bmax), 1, fp);

	fwrite(&chf.cs, sizeof(chf.cs), 1, fp);
	fwrite(&chf.ch, sizeof(chf.ch), 1, fp);

	int tmp = 0;
	if (chf.cells) tmp |= 1;
	if (chf.spans) tmp |= 2;
	if (chf.dist) tmp |= 4;
	if (chf.areas) tmp |= 8;

	fwrite(&tmp, sizeof(tmp), 1, fp);

	if (chf.cells)
		fwrite(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height, 1, fp);
	if (chf.spans)
		fwrite(chf.spans, sizeof(rcCompactSpan)*chf.spanCount, 1, fp);
	if (chf.dist)
		fwrite(chf.dist, sizeof(unsigned short)*chf.spanCount, 1, fp);
	if (chf.areas)
		fwrite(chf.areas, sizeof(unsigned char)*chf.spanCount, 1, fp);

	fclose(fp);

	return true;
}

bool duReadCompactHeightfield(struct rcCompactHeightfield& chf, const char* filepath)
{
	FILE* fp = fopen(filepath, "rb");
	if (!fp)
	{
		printf("duReadCompactHeightfield: Could not open '%s' for reading.\n", filepath);
		return false;
	}
	int magic = 0;
	int version = 0;
	
	fread(&magic, sizeof(magic), 1, fp);
	fread(&version, sizeof(version), 1, fp);
	
	if (magic != CHF_MAGIC)
	{
		printf("duReadCompactHeightfield: Bad voodoo.\n");
		fclose(fp);
		return false;
	}
	if (version != CHF_VERSION)
	{
		printf("duReadCompactHeightfield: Bad version.\n");
		fclose(fp);
		return false;
	}
	
	fread(&chf.width, sizeof(chf.width), 1, fp);
	fread(&chf.height, sizeof(chf.height), 1, fp);
	fread(&chf.spanCount, sizeof(chf.spanCount), 1, fp);
	
	fread(&chf.walkableHeight, sizeof(chf.walkableHeight), 1, fp);
	fread(&chf.walkableClimb, sizeof(chf.walkableClimb), 1, fp);
	
	fread(&chf.maxDistance, sizeof(chf.maxDistance), 1, fp);
	fread(&chf.maxRegions, sizeof(chf.maxRegions), 1, fp);
	
	fread(chf.bmin, sizeof(chf.bmin), 1, fp);
	fread(chf.bmax, sizeof(chf.bmax), 1, fp);
	
	fread(&chf.cs, sizeof(chf.cs), 1, fp);
	fread(&chf.ch, sizeof(chf.ch), 1, fp);
	
	int tmp = 0;
	fread(&tmp, sizeof(tmp), 1, fp);
	
	if (tmp & 1)
	{
		chf.cells = new rcCompactCell[chf.width*chf.height];
		if (!chf.cells)
		{
			printf("duReadCompactHeightfield: Could not alloc cells (%d)\n", chf.width*chf.height);
			fclose(fp);
			return false;
		}
		fread(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height, 1, fp);
	}
	if (tmp & 2)
	{
		chf.spans = new rcCompactSpan[chf.spanCount];
		if (!chf.spans)
		{
			printf("duReadCompactHeightfield: Could not alloc spans (%d)\n", chf.spanCount);
			fclose(fp);
			return false;
		}
		fread(chf.spans, sizeof(rcCompactSpan)*chf.spanCount, 1, fp);
	}
	if (tmp & 4)
	{
		chf.dist = new unsigned short[chf.spanCount];
		if (!chf.dist)
		{
			printf("duReadCompactHeightfield: Could not alloc dist (%d)\n", chf.spanCount);
			fclose(fp);
			return false;
		}
		fread(chf.dist, sizeof(unsigned short)*chf.spanCount, 1, fp);
	}
	if (tmp & 8)
	{
		chf.areas = new unsigned char[chf.spanCount];
		if (!chf.areas)
		{
			printf("duReadCompactHeightfield: Could not alloc areas (%d)\n", chf.spanCount);
			fclose(fp);
			return false;
		}
		fread(chf.areas, sizeof(unsigned char)*chf.spanCount, 1, fp);
	}
	
	fclose(fp);
	
	return true;
}

