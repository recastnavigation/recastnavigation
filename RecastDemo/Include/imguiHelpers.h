#pragma once

#include <imgui.h>
#include <stdio.h>

#include "AppData.h"

extern AppData app;

inline void DrawScreenspaceText(float x, float y, ImU32 color, const char* text, bool centered = false)
{
	if (centered)
	{
		const ImVec2 textSize = ImGui::CalcTextSize(text);
		x -= textSize.x * 0.5f;
	}

	ImGui::GetForegroundDrawList()->AddText({x, y}, color, text);
}

inline void DrawWorldspaceText(
	double* proj,
	double* model,
	int* view,
	float x,
	float y,
	float z,
	ImU32 color,
	const char* text,
	bool centered = false)
{
	float imguiX;
	float imguiY;
	app.WorldToScreen(x, y, z, &imguiX, &imguiY);

	if (centered)
	{
		const ImVec2 textSize = ImGui::CalcTextSize(text);
		imguiX -= textSize.x * 0.5f;
	}

	ImGui::GetForegroundDrawList()->AddText({imguiX, imguiY}, color, text);
}

inline void DrawFloatSlider(
	float* value,
	float min,
	float max,
	const char* id,
	const char* label,
	const char* valueFormat = "%.2f")
{
	// Draw the slider
	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
	ImGui::SliderFloat(id, value, min, max, "");
	ImGui::PopItemWidth();

	// Get the bounds of the slider
	ImVec2 sliderMin = ImGui::GetItemRectMin();
	ImVec2 sliderMax = ImGui::GetItemRectMax();
	ImVec2 sliderSize = ImGui::GetItemRectSize();

	ImDrawList* drawList = ImGui::GetWindowDrawList();
	ImU32 color = ImGui::GetColorU32(ImGuiCol_Text);

	// Draw the label left-aligned inside the slider
	constexpr int textPadding = 6;
	ImVec2 labelSize = ImGui::CalcTextSize(label);
	float textY = sliderMin.y + (sliderSize.y - labelSize.y) * 0.5f;
	drawList->AddText(ImVec2(sliderMin.x + textPadding, textY), color, label);

	// Draw the value label right-aligned inside the slider
	char valueLabel[32];
	snprintf(valueLabel, sizeof(valueLabel), valueFormat, *value);
	ImVec2 valueLabelSize = ImGui::CalcTextSize(valueLabel);
	drawList->AddText(ImVec2(sliderMax.x - valueLabelSize.x - textPadding, textY), color, valueLabel);
}

inline void DrawIntSlider(int* value, int min, int max, const char* id, const char* label, const char* valueFormat = "%d")
{
	// Draw the slider
	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
	ImGui::SliderInt(id, value, min, max, "");
	ImGui::PopItemWidth();

	// Get the bounds of the slider
	ImVec2 sliderMin = ImGui::GetItemRectMin();
	ImVec2 sliderMax = ImGui::GetItemRectMax();
	ImVec2 sliderSize = ImGui::GetItemRectSize();

	ImDrawList* drawList = ImGui::GetWindowDrawList();
	ImU32 color = ImGui::GetColorU32(ImGuiCol_Text);

	// Draw the label left-aligned inside the slider
	constexpr int textPadding = 6;
	ImVec2 labelSize = ImGui::CalcTextSize(label);
	float textY = sliderMin.y + (sliderSize.y - labelSize.y) * 0.5f;
	drawList->AddText(ImVec2(sliderMin.x + textPadding, textY), color, label);

	// Draw the value label right-aligned inside the slider
	char valueLabel[32];
	snprintf(valueLabel, sizeof(valueLabel), valueFormat, *value);
	ImVec2 valueLabelSize = ImGui::CalcTextSize(valueLabel);
	drawList->AddText(ImVec2(sliderMax.x - valueLabelSize.x - textPadding, textY), color, valueLabel);
}

inline void DrawRightAlignedText(const char* format, ...)
{
	static char text[2048];

	va_list args;
	va_start(args, format);
	vsnprintf(text, sizeof(text), format, args);
	va_end(args);

	float textWidth = ImGui::CalcTextSize(text).x;
	float parentWidth = ImGui::GetContentRegionAvail().x;

	ImGui::SetCursorPosX(ImGui::GetCursorPosX() + parentWidth - textWidth);
	ImGui::Text("%s", text);
}
