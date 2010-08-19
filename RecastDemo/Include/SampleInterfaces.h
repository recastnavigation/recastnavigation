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

#include "DebugDraw.h"
#include "Recast.h"
#include "RecastDump.h"

// These are example implementations of various interfaces used in Recast and Detour.

// Recast build context.
class BuildContext : public rcBuildContext
{
	int m_buildTime[RC_MAX_TIMES];

	static const int MAX_MESSAGES = 1000;
	const char* m_messages[MAX_MESSAGES];
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 8000;
	char m_textPool[TEXT_POOL_SIZE];
	int m_textPoolSize;
	
public:
	BuildContext();
	~BuildContext();
	
	// Get current time in platform specific units.
	virtual rcTimeVal getTime();
	// Returns time passed from 'start' to 'end' in microseconds.
	virtual int getDeltaTimeUsec(const rcTimeVal start, const rcTimeVal end);
	
	// Resets log.
	virtual void resetLog();
	// Logs a message.
	virtual void log(const rcLogCategory category, const char* format, ...);
	// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	// Returns number of log messages.
	int getLogCount() const;
	// Returns log message text.
	const char* getLogText(const int i) const;
	
	// Resets build time collecting.
	virtual void resetBuildTimes();
	// Reports build time of specified label for accumulation.
	virtual void reportBuildTime(const rcBuilTimeLabel label, const int time);
	// Returns accumulated build time for specified label, or -1 if no time was reported.
	virtual int getBuildTime(const rcBuilTimeLabel label);
};

// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
public:
	virtual void depthMask(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void end();
};

// stdio file implementation.
class FileIO : public duFileIO
{
	FILE* m_fp;
	int m_mode;
public:
	FileIO();
	virtual ~FileIO();
	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	virtual bool isWriting() const;
	virtual bool isReading() const;
	virtual bool write(const void* ptr, const size_t size);
	virtual bool read(void* ptr, const size_t size);
};
