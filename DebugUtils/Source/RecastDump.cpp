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
			if (p[j] == 0xffff) break;
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
