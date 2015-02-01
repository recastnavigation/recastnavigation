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

#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include <vector>
#include <string>

using std::vector;
using std::string;

class SlideShow
{
public:
	SlideShow(string path);
	~SlideShow();

	void nextSlide();
	void prevSlide();
	void setSlide(int n);
	void updateAndDraw(float dt, const float w, const float h);

private:
	void purgeImage();
	bool loadImage(const char* path);

	vector<string> m_files;
	string m_path;

	int m_width;
	int m_height;
	unsigned int m_texId;

	bool m_showCurSlide;
	float m_slideAlpha;
	int m_curSlide;
	int m_nextSlide;
};

#endif // SLIDESHOW_H
