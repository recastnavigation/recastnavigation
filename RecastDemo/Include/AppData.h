#pragma once

#include "InputGeom.h"
#include "SDL.h"
#include "Sample.h"
#include "TestCase.h"

struct AppData
{
	SDL_Window* window;
	SDL_GLContext glContext;

	double projectionMatrix[16];
	int viewport[4];

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

	~AppData();
	void resetCamera();
	void updateWindowSize();
	void updateUIScale() const;
	void worldToScreen(float x, float y, float z, float* screenX, float* screenY) const;
};
