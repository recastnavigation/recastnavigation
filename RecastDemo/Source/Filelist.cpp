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

#include "Filelist.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#endif

void FileList::Add(const char* path)
{
	if (size >= FileList::MAX_FILES)
		return;
	
	int n = strlen(path);
	files[size] = new char[n+1];
	strcpy(files[size], path);
	size++;
}

void FileList::Clear()
{
	for (int i = 0; i < size; ++i)
		delete [] files[i];
	size = 0;
}

FileList::FileList()
: size(0)
{
	for (int i = 0; i < MAX_FILES; ++i)
		files[i] = 0;
}

FileList::~FileList()
{
	Clear();
}

static int cmp(const void* a, const void* b)
{
	return strcmp(*(const char**)a, *(const char**)b);
}
	
void FileList::scanDirectory(const char* path, const char* ext)
{
	Clear();
	
#ifdef WIN32
	_finddata_t dir;
	char pathWithExt[260];
	sprintf_s(pathWithExt, "%s/*%s", path, ext);

	long fh = _findfirst(pathWithExt, &dir);
	if (fh == -1L)
	{
		return;
	}
	
	do
	{
		Add(dir.name);
	}
	while (_findnext(fh, &dir) == 0);
	_findclose(fh);
#else
	dirent* current = 0;
	DIR* dp = opendir(path);
	if (!dp)
	{
		return;
	}
	
	while ((current = readdir(dp)) != 0)
	{
		int len = strlen(current->d_name);
		if (len > 4 && strncmp(current->d_name+len-4, ext, 4) == 0)
		{
			Add(current->d_name);
		}
	}
	closedir(dp);
#endif

	if (size > 1)
	{
		qsort(files, size, sizeof(char*), cmp);
	}
}