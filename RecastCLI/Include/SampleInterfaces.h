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

#include <cstdio>
#include "Recast.h"
#include "RecastDump.h"
#include "PerfTimer.h"

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext final : public rcContext
{
	TimeVal m_startTime[RC_MAX_TIMERS]{};
	TimeVal m_accTime[RC_MAX_TIMERS]{};

	static const int MAX_MESSAGES = 1000;
	const char* m_messages[MAX_MESSAGES]{};
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 8000;
	char m_textPool[TEXT_POOL_SIZE]{};
	int m_textPoolSize;
	
public:
	BuildContext();
	
	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...)const;
	/// Returns number of log messages.
	 int getLogCount() const;
	/// Returns log message text.
	 const char* getLogText(int i) const;
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	void doResetLog() override;
	void doLog(rcLogCategory category, const char* msg, int len) override;
	void doResetTimers() override;
	void doStartTimer(rcTimerLabel label) override;
	void doStopTimer(rcTimerLabel label) override;
	 int doGetAccumulatedTime(rcTimerLabel label) const override;
	///@}
};

/// stdio file implementation.
class FileIO final : public duFileIO
{
	FILE* m_fp;
	int m_mode;
public:
	FileIO();
	~FileIO() override;
	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	 bool isWriting() const override;
	 bool isReading() const override;
	bool write(const void* ptr, size_t size) override;
	bool read(void* ptr, size_t size) override;
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	FileIO(const FileIO&);
	FileIO& operator=(const FileIO&);
};

#endif // SAMPLEINTERFACES_H

