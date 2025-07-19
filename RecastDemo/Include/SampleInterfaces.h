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

#pragma once

#include "DebugDraw.h"
#include "PerfTimer.h"
#include "Recast.h"
#include "RecastDump.h"

#include <array>
#include <cstdio>
#include <string>
#include <vector>

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
	std::array<TimeVal, RC_MAX_TIMERS> startTime;
	std::array<TimeVal, RC_MAX_TIMERS> accTime;

	std::vector<std::string> logMessages;

public:
	BuildContext();

	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;

protected:
	void doResetLog() override;
	void doLog(rcLogCategory category, const char* msg, const int len) override;
	void doResetTimers() override;
	void doStartTimer(rcTimerLabel label) override;
	void doStopTimer(rcTimerLabel label) override;
	int doGetAccumulatedTime(rcTimerLabel label) const override;
};

/// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
public:
	virtual void depthMask(bool state);
	virtual void texture(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void vertex(const float* pos, unsigned int color, const float* uv);
	virtual void vertex(float x, float y, float z, unsigned int color);
	virtual void vertex(float x, float y, float z, unsigned int color, float u, float v);
	virtual void end();
};

/// stdio file implementation.
class FileIO : public duFileIO
{
public:
	FileIO() = default;
	FileIO(const FileIO&) = delete;
	FileIO& operator=(const FileIO&) = delete;
	FileIO(FileIO&&) = default;
	FileIO& operator=(FileIO&&) = default;
	virtual ~FileIO();

	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	virtual bool isWriting() const;
	virtual bool isReading() const;
	virtual bool write(const void* ptr, const size_t size);
	virtual bool read(void* ptr, const size_t size);
	size_t getFileSize();

	static void scanDirectory(const std::string& path, const std::string& ext, std::vector<std::string>& fileList);
private:
	FILE* fp = nullptr;
	enum class Mode { none, reading, writing };
	Mode mode = Mode::none;
};
