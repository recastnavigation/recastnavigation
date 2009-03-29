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

#ifndef IMGUI_H
#define IMGUI_H

#define GENID ((__LINE__) << 16)
#define GENID1(x) ((__LINE__) << 16 | (x))

void imguiBeginFrame();
void imguiEndFrame();
void imguiRender(void (*drawText)(int x, int y, int dir, const char* text, unsigned int col));

bool imguiBeginScrollArea(unsigned int id, const char* name, int x, int y, int w, int h, int* scroll);
void imguiEndScrollArea();

void imguiIndent();
void imguiUnindent();
void imguiSeparator();

bool imguiButton(unsigned int id, const char* text);
bool imguiItem(unsigned int id, const char* text);
bool imguiCheck(unsigned int id, const char* text, bool checked);
bool imguiCollapse(unsigned int id, const char* text, bool checked);
void imguiLabel(unsigned int id, const char* text);
void imguiValue(unsigned int id, const char* text);
bool imguiSlider(unsigned int id, const char* text, float* val, float vmin, float vmax, float vinc);


#endif // IMGUI_H