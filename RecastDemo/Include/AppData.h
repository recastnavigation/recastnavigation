#pragma once

#include "InputGeom.h"
#include "Recast.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "Sample.h"
#include "TestCase.h"

#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_sdl2.h>

struct AppData
{
	SDL_Window* window;
	SDL_GLContext glContext;

	GLdouble projectionMatrix[16];
	GLint viewport[4];

	// Drawable width vs logical width (important for high-dpi screens)
	int width;
	int height;
	int drawableWidth;
	int drawableHeight;

	// Recast data, samples, and test cases
	BuildContext buildContext;
	InputGeom* inputGeometry = nullptr;
	Sample* sample = nullptr;
	TestCase* testCase = nullptr;

	// Time
	float timeAcc = 0.0f;
	Uint32 prevFrameTime = 0;

	// Input
	int mousePos[2]{0, 0};
	int origMousePos[2]{0, 0};  // Used to compute mouse movement totals across frames.

	// Camera
	float cameraEulers[2]{45, -45};
	float cameraPos[3] = {0, 0, 0};  // world space
	float camr = 1000;
	float origCameraEulers[2] = {0, 0};  // Used to compute rotational changes across frames.
	float scrollZoom = 0;

	// Movement
	float moveFront = 0.0f;
	float moveBack = 0.0f;
	float moveLeft = 0.0f;
	float moveRight = 0.0f;
	float moveUp = 0.0f;
	float moveDown = 0.0f;

	// Input state
	bool isRotatingCamera = false;
	bool movedDuringRotate = false;
	bool mouseOverMenu = false;

	// Raycasts
	float rayStart[3];  // world space
	float rayEnd[3];    // world space

	// UI state
	int sampleIndex = -1;
	std::string meshName = "Choose Mesh...";
	bool showMenu = true;
	bool showLog = false;
	bool showTools = true;
	bool showTestCases = false;
	int logScroll = 0;

	// Files
	std::vector<std::string> files;
	std::string meshesFolder = "Meshes";
	std::string testCasesFolder = "TestCases";

	// Markers
	float markerPosition[3] = {0, 0, 0};
	bool markerPositionSet = false;

	~AppData()
	{
		delete sample;
		sample = nullptr;
		delete inputGeometry;
		inputGeometry = nullptr;
	}

	void resetCamera()
	{
		// Reset camera and fog to match the mesh bounds.
		if (inputGeometry)
		{
			const float* boundsMin = inputGeometry->getNavMeshBoundsMin();
			const float* boundsMax = inputGeometry->getNavMeshBoundsMax();

			camr = sqrtf(
					   rcSqr(boundsMax[0] - boundsMin[0]) + rcSqr(boundsMax[1] - boundsMin[1]) +
					   rcSqr(boundsMax[2] - boundsMin[2])) /
			       2;
			cameraPos[0] = (boundsMax[0] + boundsMin[0]) / 2 + camr;
			cameraPos[1] = (boundsMax[1] + boundsMin[1]) / 2 + camr;
			cameraPos[2] = (boundsMax[2] + boundsMin[2]) / 2 + camr;
			camr *= 3;
		}
		cameraEulers[0] = 45;
		cameraEulers[1] = -45;
		glFogf(GL_FOG_START, camr * 0.1f);
		glFogf(GL_FOG_END, camr * 1.25f);
	}

	void UpdateWindowSize()
	{
		SDL_GetWindowSize(window, &width, &height);
		SDL_GL_GetDrawableSize(window, &drawableWidth, &drawableHeight);

		glViewport(0, 0, drawableWidth, drawableHeight);
		glGetIntegerv(GL_VIEWPORT, viewport);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, static_cast<float>(width) / static_cast<float>(height), 1.0f, camr);
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
	}

	void UpdateUIScale()
	{
		ImGuiIO& io = ImGui::GetIO();
		io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));
		io.DisplayFramebufferScale = ImVec2(
			static_cast<float>(drawableWidth) / static_cast<float>(width),
			static_cast<float>(drawableHeight) / static_cast<float>(height));
	}

	void WorldToScreen(float x, float y, float z, float* screenX, float* screenY)
	{
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);

		GLdouble winX, winY, winZ;
		gluProject(x, y, z, modelviewMatrix, projectionMatrix, viewport, &winX, &winY, &winZ);

		float dpiScaleX = float(drawableWidth) / float(width);
		float dpiScaleY = float(drawableHeight) / float(height);

		*screenX = winX / dpiScaleX;
		*screenY = height - (winY / dpiScaleY);
	}
};
