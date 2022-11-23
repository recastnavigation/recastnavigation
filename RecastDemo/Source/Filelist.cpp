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

#include <algorithm>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#	include <cstring>
#endif

using std::vector;
using std::string;

void scanDirectoryAppend(const string& path, const string& ext, vector<string>& filelist)
{
#ifdef WIN32
	string pathWithExt = path + "/*" + ext;
	
	_finddata_t dir;
	intptr_t fh = _findfirst(pathWithExt.c_str(), &dir);
	if (fh == -1L)
	{
		return;
	}
	
	do
	{
		filelist.push_back(dir.name);
	}
	while (_findnext(fh, &dir) == 0);
	_findclose(fh);
#else
	dirent* current = 0;
	DIR* dp = opendir(path.c_str());
	if (!dp)
	{
		return;
	}
	
	size_t extLen = strlen(ext.c_str());
	while ((current = readdir(dp)) != 0)
	{
		size_t len = strlen(current->d_name);
		if (len > extLen && strncmp(current->d_name + len - extLen, ext.c_str(), extLen) == 0)
		{
			filelist.push_back(current->d_name);
		}
	}
	closedir(dp);
#endif
	
	// Sort the list of files alphabetically.
	std::sort(filelist.begin(), filelist.end());
}

void scanDirectory(const string& path, const string& ext, vector<string>& filelist)
{
	filelist.clear();
	scanDirectoryAppend(path, ext, filelist);
}
