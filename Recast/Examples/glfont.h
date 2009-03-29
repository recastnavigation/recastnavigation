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

#ifndef GLFONT_H
#define GLFONT_H

class GLFont
{
public:
	GLFont(int renderVerts = 4096);
	~GLFont();

	bool	create(const char* fileName);

	int		getFontSize() const;
	int		getDescender() const;
	int		getAscender() const;
	float	getLineHeight() const;

	float	getTextLength(const char* text, float size = -1, float tracking = 0);
	void	drawText(float x, float y, const char* text,
					 unsigned int col, float size = -1, float tracking = 0);

	void	render();

	void	debugDraw();

	static unsigned int RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);

private:
	bool createFontFromFontData(unsigned char* fd);

	struct KerningPair
	{
		inline KerningPair() {}
		inline KerningPair(unsigned char c_, float dx_) : dx(dx_), c(c_) {}
		inline void Set(unsigned char c_, float dx_) { dx = dx_; c = c_; }
		float dx;
		unsigned char c, pad[3];
	};

	struct CachedGlyph
	{
		inline CachedGlyph() : w(0), h(0), ox(0), oy(0), tx(0), ty(0), adv(0.0f), nkern(0), kern(0) {}
		int w, h;
		int ox, oy;
		int tx, ty;
		float adv;
		int nkern;
		union
		{
			KerningPair* kern;
			int kernOffset;
		};
	};

	struct FontData
	{
		unsigned int	endian;
		unsigned int	version;
		unsigned int	dataSize;
		unsigned int	kernOffset;
		unsigned int	textureOffset;
		int				fontSize;
		unsigned int	texWidth;
		unsigned int	texHeight;
		int				numMipmaps;
		int				ascender;
		int				descender;
		int				lineHeight;
		int				charMin;
		int				charCount;
		CachedGlyph		glyphs[1];
	};

	FontData*		m_fd;
	unsigned int	m_texId;

	struct RenderVertex
	{
		inline void set(float x_, float y_, float u_, float v_, unsigned int c) { x=x_; y=y_; u=u_; v=v_; col=c; }
		float x, y, u, v;
		unsigned int col;
	};
	RenderVertex*	m_verts;
	int				m_nverts;
	const int		m_maxVerts;
};


#endif // GLFONT_H
