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

#ifndef SAMPLEINTERFACES_H
#define SAMPLEINTERFACES_H

#include "DebugDraw.h"
#include "Recast.h"
#include "RecastDump.h"
#include "PerfTimer.h"
#include "RecastModernCpp.h"

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];

	static const int MAX_MESSAGES = 1000;
	const char* m_messages[MAX_MESSAGES];
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 8000;
	char m_textPool[TEXT_POOL_SIZE];
	int m_textPoolSize;
	
public:
	BuildContext();
	
	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog() RC_OVERRIDE;
	virtual void doLog(const rcLogCategory category, const char* msg, const int len) RC_OVERRIDE;
	virtual void doResetTimers() RC_OVERRIDE;
	virtual void doStartTimer(const rcTimerLabel label) RC_OVERRIDE;
	virtual void doStopTimer(const rcTimerLabel label) RC_OVERRIDE;
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const RC_OVERRIDE;
	///@}
};

/// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
public:
	virtual void depthMask(bool state) RC_OVERRIDE;
	virtual void texture(bool state) RC_OVERRIDE;
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f) RC_OVERRIDE;
	virtual void vertex(const float* pos, unsigned int color) RC_OVERRIDE;
	virtual void vertex(const float x, const float y, const float z, unsigned int color) RC_OVERRIDE;
	virtual void vertex(const float* pos, unsigned int color, const float* uv) RC_OVERRIDE;
	virtual void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v) RC_OVERRIDE;
	virtual void end() RC_OVERRIDE;
};

/// stdio file implementation.
class FileIO : public duFileIO
{
	FILE* m_fp;
	int m_mode;
public:
	FileIO();
	virtual ~FileIO() RC_OVERRIDE;
	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	virtual bool isWriting() const RC_OVERRIDE;
	virtual bool isReading() const RC_OVERRIDE;
	virtual bool write(const void* ptr, const size_t size) RC_OVERRIDE;
	virtual bool read(void* ptr, const size_t size) RC_OVERRIDE;
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	FileIO(const FileIO&);
	FileIO& operator=(const FileIO&);
};

#endif // SAMPLEINTERFACES_H

