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

static void fileListAdd(FileList& list, const char* path)
{
	if (list.size >= FileList::MAX_FILES)
		return;
	int n = strlen(path);
	list.files[list.size] = new char[n+1];
	strcpy(list.files[list.size], path);
	list.size++;
}

static void fileListClear(FileList& list)
{
	for (int i = 0; i < list.size; ++i)
		delete [] list.files[i];
	list.size = 0;
}

FileList::FileList() : size(0)
{
	memset(files, 0, sizeof(char*)*MAX_FILES);
}

FileList::~FileList()
{
	fileListClear(*this);
}

static int cmp(const void* a, const void* b)
{
	return strcmp(*(const char**)a, *(const char**)b);
}
	
void scanDirectory(const char* path, const char* ext, FileList& list)
{
	fileListClear(list);
	
#ifdef WIN32
	_finddata_t dir;
	char pathWithExt[260];
	long fh;
	strcpy(pathWithExt, path);
	strcat(pathWithExt, "/*");
	strcat(pathWithExt, ext);
	fh = _findfirst(pathWithExt, &dir);
	if (fh == -1L)
		return;
	do
	{
		fileListAdd(list, dir.name);
	}
	while (_findnext(fh, &dir) == 0);
	_findclose(fh);
#else
	dirent* current = 0;
	DIR* dp = opendir(path);
	if (!dp)
		return;
	
	while ((current = readdir(dp)) != 0)
	{
		int len = strlen(current->d_name);
		if (len > 4 && strncmp(current->d_name+len-4, ext, 4) == 0)
		{
			fileListAdd(list, current->d_name);
		}
	}
	closedir(dp);
#endif

	if (list.size > 1)
		qsort(list.files, list.size, sizeof(char*), cmp);
}
