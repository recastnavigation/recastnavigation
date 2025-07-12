#pragma once

#include <imgui.h>

inline void DrawScreenspaceText(float x, float y, ImU32 color, const char* text, bool centered = false)
{
	if (centered)
	{
		const ImVec2 textSize = ImGui::CalcTextSize(text);
		x -= textSize.x * 0.5f;
	}

	ImGui::GetForegroundDrawList()->AddText({x, y}, color, text);
}

