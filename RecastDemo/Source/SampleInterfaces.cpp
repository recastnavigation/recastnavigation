#include "SampleInterfaces.h"

#include "PerfTimer.h"
#include "Recast.h"
#include "SDL_opengl.h"

#include <cstdarg>
#include <cstdio>

#ifdef WIN32
#	define snprintf _snprintf
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext()
{
	resetTimers();
}

void BuildContext::doResetLog()
{
	logMessages.clear();
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (len == 0)
	{
		return;
	}

	logMessages.emplace_back();
	std::string& message = logMessages[logMessages.size() - 1];
	message.push_back((char)category);
	message.append(msg);
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
	{
		accTime[i] = -1;
	}
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
	startTime[label] = getPerfTime();
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal endTime = getPerfTime();
	const TimeVal deltaTime = endTime - startTime[label];
	if (accTime[label] == -1)
	{
		accTime[label] = deltaTime;
	}
	else
	{
		accTime[label] += deltaTime;
	}
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return getPerfTimeUsec(accTime[label]);
}

void BuildContext::dumpLog(const char* format, ...)
{
	// Print header.
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");

	// Print messages
	const int TAB_STOPS[4] = {28, 36, 44, 52};
	for (int i = 0; i < static_cast<int>(logMessages.size()); ++i)
	{
		std::string& message = logMessages[i];
		const char* msg = message.c_str() + 1;
		int n = 0;
		while (*msg)
		{
			if (*msg == '\t')
			{
				int count = 1;
				for (int j = 0; j < 4; ++j)
				{
					if (n < TAB_STOPS[j])
					{
						count = TAB_STOPS[j] - n;
						break;
					}
				}
				while (--count)
				{
					putchar(' ');
					n++;
				}
			}
			else
			{
				putchar(*msg);
				n++;
			}
			msg++;
		}
		putchar('\n');
	}
}

int BuildContext::getLogCount() const
{
	return static_cast<int>(logMessages.size());
}

const char* BuildContext::getLogText(const int i) const
{
	return logMessages[i].c_str() + 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

class GLCheckerTexture
{
	unsigned int m_texId = 0;

public:
	~GLCheckerTexture()
	{
		if (m_texId != 0)
		{
			glDeleteTextures(1, &m_texId);
		}
	}

	void bind()
	{
		if (m_texId != 0)
		{
			glBindTexture(GL_TEXTURE_2D, m_texId);
		}
		else
		{
			// Create checker pattern.
			const unsigned int col0 = duRGBA(215, 215, 215, 255);
			const unsigned int col1 = duRGBA(255, 255, 255, 255);
			static const int TSIZE = 64;
			unsigned int data[TSIZE * TSIZE];

			glGenTextures(1, &m_texId);
			glBindTexture(GL_TEXTURE_2D, m_texId);

			int level = 0;
			int size = TSIZE;
			while (size > 0)
			{
				for (int y = 0; y < size; ++y)
				{
					for (int x = 0; x < size; ++x)
					{
						data[x + y * size] = (x == 0 || y == 0) ? col0 : col1;
					}
				}
				glTexImage2D(GL_TEXTURE_2D, level, GL_RGBA, size, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
				size /= 2;
				level++;
			}

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		}
	}
};
static GLCheckerTexture g_tex;

void DebugDrawGL::depthMask(bool state)
{
	glDepthMask(state ? GL_TRUE : GL_FALSE);
}

void DebugDrawGL::texture(bool state)
{
	if (state)
	{
		glEnable(GL_TEXTURE_2D);
		g_tex.bind();
	}
	else
	{
		glDisable(GL_TEXTURE_2D);
	}
}

void DebugDrawGL::begin(duDebugDrawPrimitives prim, float size)
{
	switch (prim)
	{
	case DU_DRAW_POINTS:
		glPointSize(size);
		glBegin(GL_POINTS);
		break;
	case DU_DRAW_LINES:
		glLineWidth(size);
		glBegin(GL_LINES);
		break;
	case DU_DRAW_TRIS:
		glBegin(GL_TRIANGLES);
		break;
	case DU_DRAW_QUADS:
		glBegin(GL_QUADS);
		break;
	};
}

void DebugDrawGL::vertex(const float* pos, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3fv(pos);
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3f(x, y, z);
}

void DebugDrawGL::vertex(const float* pos, unsigned int color, const float* uv)
{
	glColor4ubv((GLubyte*)&color);
	glTexCoord2fv(uv);
	glVertex3fv(pos);
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v)
{
	glColor4ubv((GLubyte*)&color);
	glTexCoord2f(u, v);
	glVertex3f(x, y, z);
}

void DebugDrawGL::end()
{
	glEnd();
	glLineWidth(1.0f);
	glPointSize(1.0f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

FileIO::~FileIO()
{
	if (fp)
	{
		fclose(fp);
	}
}

bool FileIO::openForWrite(const char* path)
{
	if (fp)
	{
		return false;
	}
	fp = fopen(path, "wb");
	if (!fp)
	{
		return false;
	}
	mode = 1;
	return true;
}

bool FileIO::openForRead(const char* path)
{
	if (fp)
	{
		return false;
	}
	fp = fopen(path, "rb");
	if (!fp)
	{
		return false;
	}
	mode = 2;
	return true;
}

bool FileIO::isWriting() const
{
	return mode == 1;
}

bool FileIO::isReading() const
{
	return mode == 2;
}

bool FileIO::write(const void* ptr, const size_t size)
{
	if (!fp || mode != 1)
	{
		return false;
	}
	fwrite(ptr, size, 1, fp);
	return true;
}

bool FileIO::read(void* ptr, const size_t size)
{
	if (!fp || mode != 2)
	{
		return false;
	}
	size_t readLen = fread(ptr, size, 1, fp);
	return readLen == 1;
}
