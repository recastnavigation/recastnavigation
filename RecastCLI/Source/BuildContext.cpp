#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstring>

#include "BuildContext.h"

#include <iostream>

#include "Recast.h"
#include "PerfTimer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext() :
	m_messageCount(0),
	m_textPoolSize(0)
{
	memset(m_messages, 0, sizeof(char*) * MAX_MESSAGES);

	resetTimers();
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
	m_messageCount = 0;
	m_textPoolSize = 0;
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;
	char* dst = &m_textPool[m_textPoolSize];
	const int n = TEXT_POOL_SIZE - m_textPoolSize;
	if (n < 2)
		return;
	char* cat = dst;
	char* text = dst+1;
	const int maxtext = n-1;
	// Store category
	*cat = static_cast<char>(category);
	// Store message
	const int count = rcMin(len+1, maxtext);
	memcpy(text, msg, count);
	text[count-1] = '\0';
	m_textPoolSize += 1 + count;
	m_messages[m_messageCount++] = dst;
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i) {
		m_accTime[i] == -1ll;
	}
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
	m_startTime[label] = getPerfTime();
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal endTime = getPerfTime();
	const TimeVal deltaTime = endTime - m_startTime[label];
	if (m_accTime[label] == -1)
		m_accTime[label] = deltaTime;
	else
		m_accTime[label] += deltaTime;
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return getPerfTimeUsec(m_accTime[label]);
}

void BuildContext::dumpLog(const char* format, ...) const
{
	// Print header.
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	std::cout << std::endl;

	// Print messages
	constexpr int TAB_STOPS[4] = { 28, 36, 44, 52 };
	for (int i = 0; i < MAX_MESSAGES; ++i) {
		const auto& message = m_messages[i];
		int n = 0;
		for (const char ch : message) {
			if (ch == '\t') {
				int count = 1;
				for (const int j : TAB_STOPS) {
					if (n < j) {
						count = j - n;
						break;
					}
				}
				while (count-- > 0) {
					std::cout << ' ';
					n++;
				}
			} else {
				std::cout << ch;
				n++;
			}
		}
		std::cout << '\n';
	}
	std::cout << std::flush;
}

int BuildContext::getLogCount() const
{
	return m_messageCount;
}

const char* BuildContext::getLogText(const int i) const
{
	return m_messages[i]+1;
}

