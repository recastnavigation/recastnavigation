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
#include <string>
#include <vector>

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext final : public rcContext
{
	std::array<TimeVal, RC_MAX_TIMERS> startTime;
	std::array<TimeVal, RC_MAX_TIMERS> accTime;

	std::vector<std::string> logMessages;

public:
	BuildContext();

	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	[[nodiscard]] int getLogCount() const;
	/// Returns log message text.
	[[nodiscard]] const char* getLogText(int i) const;

protected:
	void doResetLog() override;
	void doLog(rcLogCategory category, const char* msg, const int len) override;
	void doResetTimers() override;
	void doStartTimer(rcTimerLabel label) override;
	void doStopTimer(rcTimerLabel label) override;
	[[nodiscard]] int doGetAccumulatedTime(rcTimerLabel label) const override;
};

/// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
public:
	void depthMask(bool state) override;
	void texture(bool state) override;
	void begin(duDebugDrawPrimitives prim, float size = 1.0f) override;
	void vertex(const float* pos, unsigned int color) override;
	void vertex(const float* pos, unsigned int color, const float* uv) override;
	void vertex(float x, float y, float z, unsigned int color) override;
	void vertex(float x, float y, float z, unsigned int color, float u, float v) override;
	void end() override;
};

/// stdio file implementation.
class FileIO final : public duFileIO
{
public:
	FileIO() = default;
	FileIO(const FileIO&) = delete;
	FileIO& operator=(const FileIO&) = delete;
	FileIO(FileIO&&) = default;
	FileIO& operator=(FileIO&&) = default;
	~FileIO() override;

	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	[[nodiscard]] bool isWriting() const override;
	[[nodiscard]] bool isReading() const override;
	bool write(const void* ptr, size_t size) override;
	bool read(void* ptr, size_t size) override;
	size_t getFileSize() const;

	static void scanDirectory(const std::string& path, const std::string& ext, std::vector<std::string>& fileList);
private:
	FILE* fp = nullptr;
	enum class Mode { none, reading, writing };
	Mode mode = Mode::none;
};
