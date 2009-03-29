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

#include "GlFont.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <SDL_Opengl.h>
#include <stdlib.h>

GLFont::GLFont(int renderVerts) :
	m_fd(0),
	m_texId(0),
	m_verts(0),
	m_nverts(0),
	m_maxVerts(renderVerts)
{
}

GLFont::~GLFont()
{
	if (m_texId)
		glDeleteTextures(1, (GLuint*)&m_texId);
	unsigned char* data = (unsigned char*)m_fd;
	if (data)
		free(data);
	if (m_verts)
		free(m_verts);
}

bool GLFont::create(const char* fileName)
{
	unsigned char* data = 0;

	FILE* fp = fopen(fileName, "rb");
	if (!fp)
		return false;

	// Read cache file
	fseek(fp, 0, SEEK_END);
	unsigned n = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	data = (unsigned char*)malloc(n);
	fread(data, n, 1, fp);
	fclose(fp);

	if (!m_verts)
		m_verts = (RenderVertex*)malloc(m_maxVerts*sizeof(RenderVertex));

	return createFontFromFontData(data);
}

bool GLFont::createFontFromFontData(unsigned char* data)
{
	if (!data)
	{
		printf("GLFont::createFontFromFontData: No input data!\n");
		return false;
	}

	m_fd = (FontData*)data;

	// Patch kern pointers.
	for (int i = 0; i < m_fd->charCount; ++i)
		m_fd->glyphs[i].kern = (KerningPair*)((int)m_fd->glyphs[i].kernOffset + data);

	unsigned char* texData = data + m_fd->textureOffset;

	// Create textures
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, (GLuint*)&m_texId);
	glBindTexture(GL_TEXTURE_2D, m_texId);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, m_fd->texWidth, m_fd->texHeight, 0,
		GL_ALPHA, GL_UNSIGNED_BYTE, texData);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	return true;
}

int GLFont::getFontSize() const
{
	return m_fd ? m_fd->fontSize : 0;
}

int GLFont::getDescender() const
{
	return m_fd ? m_fd->descender : 0;
}

int GLFont::getAscender() const
{
	return m_fd ? m_fd->ascender : 0;
}

float GLFont::getLineHeight() const
{
	return m_fd ? m_fd->lineHeight : 0.0f;
}
	
float GLFont::getTextLength(const char* text, float size, float tracking)
{
	if (!m_texId) return 0.0f;
	if (!m_fd) return 0.0f;

	float scale = size < 0 ? 1 : size / (float)m_fd->fontSize;
	float track = scale * m_fd->ascender * tracking / 1000.0f;

	const unsigned char* src = (const unsigned char*)text;
		
	int prevc = -1;
	float len = 0.0f;
	float tx = 0.0f;
	
	for (; *src; ++src)
	{
		int	c = (int)*src - m_fd->charMin;
		if (c < 0 || c >= m_fd->charCount)
		{
			prevc = c;
			continue;
		}
			
		CachedGlyph& cg = m_fd->glyphs[c];
		if (prevc > 0 && prevc < m_fd->charCount)
		{
			CachedGlyph& prevcg = m_fd->glyphs[prevc];
			if (prevcg.nkern != 0)
			{
				for (int i = 0; i < prevcg.nkern; ++i)
				{
					if (prevcg.kern[i].c == c)
					{
						tx += prevcg.kern[i].dx * scale;
						break;
					}
				}
			}
		}

		len = tx + (cg.ox + cg.w) * scale;
				
		tx += cg.adv * scale + track;
		prevc = c;
	}

	return len;
}

void GLFont::drawText(float tx, float ty, const char* text,
				  unsigned int col, float size, float tracking)
{
	if (!m_fd) return;
	if (!m_texId) return;
	if (!m_verts) return;

	float scale = size < 0 ? 1 : size / (float)m_fd->fontSize;
	float track = scale * m_fd->ascender * tracking / 1000.0f;
	float su = 1.0f / m_fd->texWidth;
	float sv = 1.0f / m_fd->texHeight;

	const unsigned char* src = (const unsigned char*)text;
		
	RenderVertex* v = &m_verts[m_nverts];

	int prevc = -1;
	
	for (; *src; ++src)
	{
		int	c = (int)*src - m_fd->charMin;
		if (c == '\n')
		{
			ty -= getLineHeight();
			prevc = -1;
			continue;
		}
		if (c < 0 || c >= m_fd->charCount)
		{
			prevc = c;
			continue;
		}
			
		CachedGlyph& cg = m_fd->glyphs[c];
		if (prevc > 0 && prevc < m_fd->charCount)
		{
			CachedGlyph& prevcg = m_fd->glyphs[prevc];
			if (prevcg.nkern != 0)
			{
				for (int i = 0; i < prevcg.nkern; ++i)
				{
					if (prevcg.kern[i].c == c)
					{
						tx += prevcg.kern[i].dx * scale;
						break;
					}
				}
			}
		}

		float x0 = floorf(tx + (cg.ox - 1) * scale + 0.5f);
		float y0 = floorf(ty + (cg.oy - 1) * scale + 0.5f);
		float x1 = floorf(x0 + (cg.w + 2) * scale + 0.5f);
		float y1 = floorf(y0 + (cg.h + 2) * scale + 0.5f);

		float u0 = (cg.tx - 1) * su;
		float v0 = (cg.ty - 1) * sv;
		float u1 = (cg.tx + cg.w + 1) * su;
		float v1 = (cg.ty + cg.h + 1) * sv;

		if (m_nverts+6 > m_maxVerts) break;

		v->set(x0, y0, u0, v0, col); v++;
		v->set(x1, y0, u1, v0, col); v++;
		v->set(x1, y1, u1, v1, col); v++;

		v->set(x0, y0, u0, v0, col); v++;
		v->set(x1, y1, u1, v1, col); v++;
		v->set(x0, y1, u0, v1, col); v++;

		m_nverts += 6;

		tx += cg.adv * scale + track;
		prevc = c;
	}

	render();
}

void GLFont::render()
{
	if (!m_fd) return;
	if (!m_texId) return;
	if (!m_verts) return;

	// Render
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_texId);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glDisableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glVertexPointer(2, GL_FLOAT, sizeof(RenderVertex), &m_verts[0].x);
	glTexCoordPointer(2, GL_FLOAT, sizeof(RenderVertex), &m_verts[0].u);
	glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(RenderVertex), &m_verts[0].col);

	glDrawArrays(GL_TRIANGLES, 0, m_nverts);
	m_nverts = 0;

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);
}

unsigned int GLFont::RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	return (a<<24) | (b<<16) | (g<<8) | r;
/*#ifdef WIN32
	return (a<<24) | (b<<16) | (g<<8) | r;
#else
	return (r<<24) | (g<<16) | (b<<8) | a;
#endif*/
}
