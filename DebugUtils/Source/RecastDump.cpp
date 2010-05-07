//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
#include <stdarg.h>
#include "Recast.h"
#include "RecastDump.h"


duFileIO::~duFileIO()
{
	// Empty
}
	
static void ioprintf(duFileIO* io, const char* format, ...)
{
	char line[256];
	va_list ap;
	va_start(ap, format);
	const int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	if (n > 0)
		io->write(line, sizeof(char)*n);
}

bool duDumpPolyMeshToObj(rcPolyMesh& pmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshToObj: input IO not writing.\n"); 
		return false;
	}
	
	const int nvp = pmesh.nvp;
	const float cs = pmesh.cs;
	const float ch = pmesh.ch;
	const float* orig = pmesh.bmin;
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");

	ioprintf(io, "\n");
	
	for (int i = 0; i < pmesh.nverts; ++i)
	{
		const unsigned short* v = &pmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		ioprintf(io, "v %f %f %f\n", x,y,z);
	}

	ioprintf(io, "\n");

	for (int i = 0; i < pmesh.npolys; ++i)
	{
		const unsigned short* p = &pmesh.polys[i*nvp*2];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			ioprintf(io, "f %d %d %d\n", p[0]+1, p[j-1]+1, p[j]+1); 
		}
	}
	
	return true;
}

bool duDumpPolyMeshDetailToObj(rcPolyMeshDetail& dmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshDetailToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshDetailToObj: input IO not writing.\n"); 
		return false;
	}
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");
	
	ioprintf(io, "\n");

	for (int i = 0; i < dmesh.nverts; ++i)
	{
		const float* v = &dmesh.verts[i*3];
		ioprintf(io, "v %f %f %f\n", v[0],v[1],v[2]);
	}
	
	ioprintf(io, "\n");
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned short* m = &dmesh.meshes[i*4];
		const unsigned short bverts = m[0];
		const unsigned short btris = m[2];
		const unsigned short ntris = m[3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		for (int j = 0; j < ntris; ++j)
		{
			ioprintf(io, "f %d %d %d\n",
					(int)(bverts+tris[j*4+0])+1,
					(int)(bverts+tris[j*4+1])+1,
					(int)(bverts+tris[j*4+2])+1);
		}
	}
	
	return true;
}


static const int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static const int CHF_VERSION = 2;

bool duDumpCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpCompactHeightfield: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CHF_MAGIC, sizeof(CHF_MAGIC));
	io->write(&CHF_VERSION, sizeof(CHF_VERSION));
	
	io->write(&chf.width, sizeof(chf.width));
	io->write(&chf.height, sizeof(chf.height));
	io->write(&chf.spanCount, sizeof(chf.spanCount));

	io->write(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->write(&chf.walkableClimb, sizeof(chf.walkableClimb));

	io->write(&chf.maxDistance, sizeof(chf.maxDistance));
	io->write(&chf.maxRegions, sizeof(chf.maxRegions));

	io->write(chf.bmin, sizeof(chf.bmin));
	io->write(chf.bmax, sizeof(chf.bmax));

	io->write(&chf.cs, sizeof(chf.cs));
	io->write(&chf.ch, sizeof(chf.ch));

	int tmp = 0;
	if (chf.cells) tmp |= 1;
	if (chf.spans) tmp |= 2;
	if (chf.dist) tmp |= 4;
	if (chf.areas) tmp |= 8;

	io->write(&tmp, sizeof(tmp));

	if (chf.cells)
		io->write(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height);
	if (chf.spans)
		io->write(chf.spans, sizeof(rcCompactSpan)*chf.spanCount);
	if (chf.dist)
		io->write(chf.dist, sizeof(unsigned short)*chf.spanCount);
	if (chf.areas)
		io->write(chf.areas, sizeof(unsigned char)*chf.spanCount);

	return true;
}

bool duReadCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duReadCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadCompactHeightfield: input IO not reading.\n"); 
		return false;
	}

	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CHF_MAGIC)
	{
		printf("duReadCompactHeightfield: Bad voodoo.\n");
		return false;
	}
	if (version != CHF_VERSION)
	{
		printf("duReadCompactHeightfield: Bad version.\n");
		return false;
	}
	
	io->read(&chf.width, sizeof(chf.width));
	io->read(&chf.height, sizeof(chf.height));
	io->read(&chf.spanCount, sizeof(chf.spanCount));
	
	io->read(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->read(&chf.walkableClimb, sizeof(chf.walkableClimb));
	
	io->read(&chf.maxDistance, sizeof(chf.maxDistance));
	io->read(&chf.maxRegions, sizeof(chf.maxRegions));
	
	io->read(chf.bmin, sizeof(chf.bmin));
	io->read(chf.bmax, sizeof(chf.bmax));
	
	io->read(&chf.cs, sizeof(chf.cs));
	io->read(&chf.ch, sizeof(chf.ch));
	
	int tmp = 0;
	io->read(&tmp, sizeof(tmp));
	
	if (tmp & 1)
	{
		chf.cells = new rcCompactCell[chf.width*chf.height];
		if (!chf.cells)
		{
			printf("duReadCompactHeightfield: Could not alloc cells (%d)\n", chf.width*chf.height);
			return false;
		}
		io->read(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height);
	}
	if (tmp & 2)
	{
		chf.spans = new rcCompactSpan[chf.spanCount];
		if (!chf.spans)
		{
			printf("duReadCompactHeightfield: Could not alloc spans (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.spans, sizeof(rcCompactSpan)*chf.spanCount);
	}
	if (tmp & 4)
	{
		chf.dist = new unsigned short[chf.spanCount];
		if (!chf.dist)
		{
			printf("duReadCompactHeightfield: Could not alloc dist (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.dist, sizeof(unsigned short)*chf.spanCount);
	}
	if (tmp & 8)
	{
		chf.areas = new unsigned char[chf.spanCount];
		if (!chf.areas)
		{
			printf("duReadCompactHeightfield: Could not alloc areas (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.areas, sizeof(unsigned char)*chf.spanCount);
	}
	
	return true;
}

