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

#include "SDL.h"
#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include "InputGeom.h"
#include "Recast.h"
#include "Sample_SoloMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_TileMesh.h"
#include "TestCase.h"
#include "imguiHelpers.h"

#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_sdl2.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

using std::string;
using std::vector;

struct SampleItem
{
	string name;
	std::function<Sample*()> create;
};

namespace
{
SampleItem g_samples[] = {
	{"Solo Mesh",      []() { return new Sample_SoloMesh(); }     },
	{"Tile Mesh",      []() { return new Sample_TileMesh(); }     },
	{"Temp Obstacles", []() { return new Sample_TempObstacles(); }},
};
constexpr int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

constexpr float fogColor[4] = {0.32f, 0.31f, 0.30f, 1.0f};

}

struct AppData
{
	// Window & SDL
	int width;
	int height;
	SDL_Window* window;
	SDL_GLContext glContext;

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
	float cameraPos[3] = {0, 0, 0};
	float camr = 1000;
	float origCameraEulers[2] = {0, 0};  // Used to compute rotational changes across frames.

	// Movement
	float moveFront = 0.0f;
	float moveBack = 0.0f;
	float moveLeft = 0.0f;
	float moveRight = 0.0f;
	float moveUp = 0.0f;
	float moveDown = 0.0f;

	// Zoom
	float scrollZoom = 0;

	// Input state
	bool isRotatingCamera = false;
	bool movedDuringRotate = false;
	bool mouseOverMenu = false;

	// Raycasts
	float rayStart[3];
	float rayEnd[3];

	// UI
	int sampleIndex = -1;
	string meshName = "Choose Mesh...";

	// UI state
	bool showMenu = true;
	bool showLog = false;
	bool showTools = true;
	bool showTestCases = false;

	// Window scroll positions.
	int logScroll = 0;

	// Files
	vector<string> files;
	string meshesFolder = "Meshes";
	string testCasesFolder = "TestCases";

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
		const float* bmin = 0;
		const float* bmax = 0;
		if (inputGeometry)
		{
			bmin = inputGeometry->getNavMeshBoundsMin();
			bmax = inputGeometry->getNavMeshBoundsMax();
		}
		// Reset camera and fog to match the mesh bounds.
		if (bmin && bmax)
		{
			camr = sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
			cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
			cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
			cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
			camr *= 3;
		}
		cameraEulers[0] = 45;
		cameraEulers[1] = -45;
		glFogf(GL_FOG_START, camr * 0.1f);
		glFogf(GL_FOG_END, camr * 1.25f);
	}
};

int main(int /*argc*/, char** /*argv*/)
{
	AppData app;

	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL.\nError: %s\n", SDL_GetError());
		return -1;
	}

	// Use OpenGL render driver.
	SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengl");

	// Enable depth buffer.
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	// Set color channel depth.
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

	// 4x MSAA.
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_DisplayMode displayMode;
	SDL_GetCurrentDisplayMode(0, &displayMode);

	app.width = rcMin(displayMode.w, static_cast<int>(static_cast<float>(displayMode.h) * (16.0f / 9.0f))) - 80;
	app.height = displayMode.h - 80;

	// Create the SDL window with OpenGL support
	app.window = SDL_CreateWindow(
		"My App",
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		app.width,
		app.height,
		SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

	// Create the OpenGL context
	app.glContext = SDL_GL_CreateContext(app.window);
	SDL_GL_MakeCurrent(app.window, app.glContext);

	if (!app.window || !app.glContext)
	{
		printf("Could not initialize SDL opengl\nError: %s\n", SDL_GetError());
		return -1;
	}

	SDL_SetWindowPosition(app.window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls
	io.Fonts->AddFontFromFileTTF("DroidSans.ttf", 16.0f); // Size in pixels
	ImGui::PushFont(io.Fonts->Fonts[0]);
	ImGui::StyleColorsDark();



	// Set style
	float main_scale = ImGui_ImplSDL2_GetContentScaleForDisplay(0);
	ImGuiStyle& style = ImGui::GetStyle();
	style.ScaleAllSizes(main_scale);
	style.FontScaleDpi = main_scale;
	style.WindowRounding = 8.0f;
	style.FrameRounding  = 4.0f;
	style.WindowPadding  = ImVec2(10, 10);
	style.FramePadding   = ImVec2(8, 4);

	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.75f);
	style.Colors[ImGuiCol_Header] = ImVec4(1.0f, 1.0f, 1.0f, 0.5f);
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.75f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(1.0f, 1.0f, 1.0f, 0.25f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(1.0f, 0.75f, 0, 0.75f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(1.0f, 0.75f, 0, 0.37f);

	style.Colors[ImGuiCol_Button] = ImVec4(0.5f, 0.5f, 0.5f, 96.0f / 255.0f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.5f, 0.5f, 0.5f, 196.0f / 255.0f);
	//style.Colors[ImGuiCol_ButtonHovered] = ImVec4(1.0f, 196.0f / 255.0f, 0, 96.0f / 255.0f);






	// Setup Platform/Renderer backends
	ImGui_ImplSDL2_InitForOpenGL(app.window, app.glContext);
    ImGui_ImplOpenGL2_Init();

	ImGuiWindowFlags staticWindowFlags = ImGuiWindowFlags_NoMove
		//| ImGuiWindowFlags_NoResize
		| ImGuiWindowFlags_NoSavedSettings
		| ImGuiWindowFlags_NoCollapse;

	app.prevFrameTime = SDL_GetTicks();

	// Set up fog.
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, app.camr * 0.1f);
	glFogf(GL_FOG_END, app.camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);

	// OpenGL settings
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);



	//----------------------------------------------------------------------------
	FileIO::scanDirectory(app.meshesFolder, ".obj", app.files);
	app.meshName = app.files[0];
	app.inputGeometry = new InputGeom;
	app.inputGeometry->load(&app.buildContext, app.meshesFolder + "/" + app.meshName);

	app.sampleIndex = 0;
	app.sample = g_samples[0].create();
	app.sample->setContext(&app.buildContext);
	app.sample->handleMeshChanged(app.inputGeometry);

	// Reset camera and fog to match the mesh bounds.
	const float* bmin = app.inputGeometry->getNavMeshBoundsMin();
	const float* bmax = app.inputGeometry->getNavMeshBoundsMax();
	app.camr = sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
	app.cameraPos[0] = (bmax[0] + bmin[0]) / 2 + app.camr;
	app.cameraPos[1] = (bmax[1] + bmin[1]) / 2 + app.camr;
	app.cameraPos[2] = (bmax[2] + bmin[2]) / 2 + app.camr;
	app.camr *= 3;
	app.cameraEulers[0] = 45;
	app.cameraEulers[1] = -45;
	glFogf(GL_FOG_START, app.camr * 0.1f);
	glFogf(GL_FOG_END, app.camr * 1.25f);
	//----------------------------------------------------------------------------


	bool done = false;
	while (!done)
	{
		// Handle input events.
		bool processHitTest = false;
		bool processHitTestShift = false;

		// Per frame input
		app.mouseOverMenu = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			ImGui_ImplSDL2_ProcessEvent(&event);
			switch (event.type)
			{
			case SDL_KEYDOWN:
				// Handle any key presses here.
				switch (event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					done = true;
					break;
				case SDLK_t:
					app.showTestCases = true;
					app.files.clear();
					FileIO::scanDirectory(app.testCasesFolder, ".txt", app.files);
					break;
				case SDLK_TAB:
					app.showMenu = !app.showMenu;
					break;
				case SDLK_SPACE:
					if (app.sample)
					{
						app.sample->handleToggle();
					}
					break;
				case SDLK_1:
					if (app.sample)
					{
						app.sample->handleStep();
					}
					break;
				case SDLK_9:
					if (app.sample && app.inputGeometry)
					{
						BuildSettings settings;
						rcVcopy(settings.navMeshBMin, app.inputGeometry->getNavMeshBoundsMin());
						rcVcopy(settings.navMeshBMax, app.inputGeometry->getNavMeshBoundsMax());
						app.sample->collectSettings(settings);
						app.inputGeometry->saveGeomSet(&settings);
					}
					break;
				}
				break;

			case SDL_MOUSEWHEEL:
				if (!app.mouseOverMenu)
				{
					app.scrollZoom += static_cast<float>(event.wheel.y);
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				if (event.button.button == SDL_BUTTON_RIGHT && !app.mouseOverMenu)
				{
					// Rotate view
					app.isRotatingCamera = true;
					app.movedDuringRotate = false;
					app.origMousePos[0] = app.mousePos[0];
					app.origMousePos[1] = app.mousePos[1];
					app.origCameraEulers[0] = app.cameraEulers[0];
					app.origCameraEulers[1] = app.cameraEulers[1];
				}
				break;

			case SDL_MOUSEBUTTONUP:
				// Handle mouse clicks here.
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					app.isRotatingCamera = false;
					if (!app.mouseOverMenu)
					{
						if (!app.movedDuringRotate)
						{
							processHitTest = true;
							processHitTestShift = true;
						}
					}
				}
				else if (event.button.button == SDL_BUTTON_LEFT)
				{
					if (!app.mouseOverMenu)
					{
						processHitTest = true;
						processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
					}
				}

				break;

			case SDL_MOUSEMOTION:
				app.mousePos[0] = event.motion.x;
				app.mousePos[1] = app.height - 1 - event.motion.y;

				if (app.isRotatingCamera)
				{
					int dx = app.mousePos[0] - app.origMousePos[0];
					int dy = app.mousePos[1] - app.origMousePos[1];
					app.cameraEulers[0] = app.origCameraEulers[0] - static_cast<float>(dy) * 0.25f;
					app.cameraEulers[1] = app.origCameraEulers[1] + static_cast<float>(dx) * 0.25f;
					if (dx * dx + dy * dy > 3 * 3)
					{
						app.movedDuringRotate = true;
					}
				}
				break;
			case SDL_WINDOWEVENT:
			{
				if (event.window.event == SDL_WINDOWEVENT_RESIZED)
				{
					// Get the new window size
					app.width = event.window.data1;
					app.height = event.window.data2;

					// Update OpenGL viewport
					glViewport(0, 0, app.width, app.height);

					glMatrixMode(GL_PROJECTION);
					glLoadIdentity();
					constexpr float FOV = 50.0f;
					const float aspect = (float)app.width / (float)app.height;
					constexpr float zNear = 1.0f;
					const float zFar = app.camr;
					gluPerspective(FOV, aspect, zNear, zFar);
				}
			}
			break;
			case SDL_QUIT:
				done = true;
				break;

			default:
				break;
			}
		}

		Uint32 time = SDL_GetTicks();
		float dt = static_cast<float>(time - app.prevFrameTime) / 1000.0f;
		app.prevFrameTime = time;

		// Hit test mesh.
		if (processHitTest && app.inputGeometry && app.sample)
		{
			float hitTime;
			bool hit = app.inputGeometry->raycastMesh(app.rayStart, app.rayEnd, hitTime);

			if (hit)
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					app.markerPositionSet = true;
					app.markerPosition[0] = app.rayStart[0] + (app.rayEnd[0] - app.rayStart[0]) * hitTime;
					app.markerPosition[1] = app.rayStart[1] + (app.rayEnd[1] - app.rayStart[1]) * hitTime;
					app.markerPosition[2] = app.rayStart[2] + (app.rayEnd[2] - app.rayStart[2]) * hitTime;
				}
				else
				{
					float hitPos[3];
					hitPos[0] = app.rayStart[0] + (app.rayEnd[0] - app.rayStart[0]) * hitTime;
					hitPos[1] = app.rayStart[1] + (app.rayEnd[1] - app.rayStart[1]) * hitTime;
					hitPos[2] = app.rayStart[2] + (app.rayEnd[2] - app.rayStart[2]) * hitTime;
					app.sample->handleClick(app.rayStart, hitPos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					app.markerPositionSet = false;
				}
			}
		}

		// Update sample simulation.
		constexpr float SIM_RATE = 20;
		constexpr float DELTA_TIME = 1.0f / SIM_RATE;
		app.timeAcc = rcClamp(app.timeAcc + dt, -1.0f, 1.0f);
		int simIter = 0;
		while (app.timeAcc > DELTA_TIME)
		{
			app.timeAcc -= DELTA_TIME;
			if (simIter < 5 && app.sample)
			{
				app.sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		constexpr float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = static_cast<int>((MIN_FRAME_TIME - dt) * 1000.0f);
			ms = std::min(ms, 10);
			if (ms >= 0)
			{
				SDL_Delay(ms);
			}
		}

		// Set the viewport.
		glViewport(0, 0, app.width, app.height);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		// Clear the screen
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);

		// Compute the projection matrix.
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)app.width / (float)app.height, 1.0f, app.camr);
		GLdouble projectionMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);

		// Compute the modelview matrix.
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(app.cameraEulers[0], 1, 0, 0);
		glRotatef(app.cameraEulers[1], 0, 1, 0);
		glTranslatef(-app.cameraPos[0], -app.cameraPos[1], -app.cameraPos[2]);
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);

		// Get hit ray position and direction.
		GLdouble x, y, z;
		gluUnProject(app.mousePos[0], app.mousePos[1], 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		app.rayStart[0] = (float)x;
		app.rayStart[1] = (float)y;
		app.rayStart[2] = (float)z;
		gluUnProject(app.mousePos[0], app.mousePos[1], 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		app.rayEnd[0] = (float)x;
		app.rayEnd[1] = (float)y;
		app.rayEnd[2] = (float)z;

		// Handle keyboard movement.
		const Uint8* keystate = SDL_GetKeyboardState(NULL);
		app.moveFront = rcClamp(
			app.moveFront + dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);
		app.moveLeft = rcClamp(
			app.moveLeft + dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);
		app.moveBack = rcClamp(
			app.moveBack + dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);
		app.moveRight = rcClamp(
			app.moveRight + dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);
		app.moveUp = rcClamp(
			app.moveUp + dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);
		app.moveDown = rcClamp(
			app.moveDown + dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN]) ? 1.0f : -1.0f),
			0.0f,
			1.0f);

		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
		{
			keybSpeed *= 4.0f;
		}

		float moveX = (app.moveRight - app.moveLeft) * keybSpeed * dt;
		float moveY = (app.moveBack - app.moveFront) * keybSpeed * dt + app.scrollZoom * 2.0f;
		app.scrollZoom = 0;

		app.cameraPos[0] += moveX * static_cast<float>(modelviewMatrix[0]);
		app.cameraPos[1] += moveX * static_cast<float>(modelviewMatrix[4]);
		app.cameraPos[2] += moveX * static_cast<float>(modelviewMatrix[8]);

		app.cameraPos[0] += moveY * static_cast<float>(modelviewMatrix[2]);
		app.cameraPos[1] += moveY * static_cast<float>(modelviewMatrix[6]);
		app.cameraPos[2] += moveY * static_cast<float>(modelviewMatrix[10]);

		app.cameraPos[1] += (app.moveUp - app.moveDown) * keybSpeed * dt;

		// Draw the mesh
		glEnable(GL_FOG);
		if (app.sample)
		{
			app.sample->handleRender();
		}
		if (app.testCase)
		{
			app.testCase->handleRender();
		}
		glDisable(GL_FOG);

		// Draw UI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, app.width, 0, app.height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();
		ImGui::ShowDemoWindow();

		if (app.sample)
		{
			app.sample->handleRenderOverlay(projectionMatrix, modelviewMatrix, viewport);
		}
		if (app.testCase)
		{
			app.testCase->handleRenderOverlay(projectionMatrix, modelviewMatrix, viewport);
		}

		// Help text.
		if (app.showMenu)
		{
			DrawScreenspaceText(280.0f, 20.0f, IM_COL32(255, 255, 255, 128), "W/A/S/D: Move  RMB: Rotate");
		}

		bool newMeshSelected = false;
		bool newSampleSelected = false;

		if (app.showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2(static_cast<float>(app.width - 250 - 10), 10), ImGuiCond_Always);  // Position in screen space
			ImGui::SetNextWindowSize(ImVec2(350, static_cast<float>(app.height - 20)), ImGuiCond_Always);     // Size of the window
			ImGui::Begin("Properties", nullptr, staticWindowFlags);

			ImGui::Checkbox("Show Log", &app.showLog);
			ImGui::Checkbox("Show Tools", &app.showTools);

			ImGui::SeparatorText("Input Mesh");

			// Level selection dialog.
			if (ImGui::BeginCombo("##levelCombo", app.meshName.c_str(), 0))
			{
				app.files.clear();
				FileIO::scanDirectory(app.meshesFolder, ".obj", app.files);
				FileIO::scanDirectory(app.meshesFolder, ".gset", app.files);

				for (const auto& file : app.files)
				{
					const bool is_selected = (app.meshName == file);
					if (ImGui::Selectable(file.c_str(), is_selected) && !is_selected)
					{
						app.meshName = file;
						newMeshSelected = true;
					}

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
					{
						ImGui::SetItemDefaultFocus();
					}
				}
				ImGui::EndCombo();
			}

			if (app.inputGeometry)
			{
				ImGui::Text(
					"Verts: %.1fk  Tris: %.1fk",
							static_cast<float>(app.inputGeometry->getVertCount()) / 1000.0f,
							static_cast<float>(app.inputGeometry->getTriCount()) / 1000.0f);
			}

			ImGui::SeparatorText("Sample");

			if (ImGui::BeginCombo("##sampleCombo", app.sampleIndex >= 0 ? g_samples[app.sampleIndex].name.c_str() : "Choose Sample...", 0))
			{
				for (int n = 0; n < IM_ARRAYSIZE(g_samples); n++)
				{
					const bool is_selected = (app.sampleIndex == n);
					if (ImGui::Selectable(g_samples[n].name.c_str(), is_selected))
					{
						newSampleSelected = !is_selected;
						app.sampleIndex = n;
					}

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
					{
						ImGui::SetItemDefaultFocus();
					}
				}
				ImGui::EndCombo();
			}

			if (app.sample)
			{
				if (app.inputGeometry)
				{
					app.sample->handleSettings();

					if (ImGui::Button("Build"))
					{
						app.buildContext.resetLog();
						if (!app.sample->handleBuild())
						{
							app.showLog = true;
							app.logScroll = 0;
						}
						app.buildContext.dumpLog("Build log %s:", app.meshName.c_str());

						// Clear test.
						delete app.testCase;
						app.testCase = 0;
					}
				}

				ImGui::SeparatorText("Debug Settings");
				app.sample->handleDebugMode();
			}

			ImGui::End();
		}

		if (newSampleSelected)
		{
			delete app.sample;
			app.sample = g_samples[app.sampleIndex].create();
			app.sample->setContext(&app.buildContext);
			if (app.inputGeometry)
			{
				app.sample->handleMeshChanged(app.inputGeometry);
				app.resetCamera();
			}
		}

		if (newMeshSelected)
		{
			string path = app.meshesFolder + "/" + app.meshName;

			delete app.inputGeometry;
			app.inputGeometry = new InputGeom;
			if (!app.inputGeometry->load(&app.buildContext, path))
			{
				delete app.inputGeometry;
				app.inputGeometry = nullptr;

				// Destroy the sample if it already had geometry loaded, as we've just deleted it!
				if (app.sample && app.sample->getInputGeom())
				{
					delete app.sample;
					app.sample = nullptr;
				}

				app.showLog = true;
				app.logScroll = 0;
				app.buildContext.dumpLog("geom load log %s:", app.meshName.c_str());
			}
			app.resetCamera();
			if (app.sample)
			{
				app.sample->handleMeshChanged(app.inputGeometry);
			}
		}

#if 0
		// Test cases
		if (app.showTestCases)
		{
			ImGui::SetNextWindowPos(ImVec2(static_cast<float>(app.width - 10 - 250 - 10 - 200), static_cast<float>(app.height - 10 - 450)), ImGuiCond_Always);  // Position in screen space
			ImGui::SetNextWindowSize(ImVec2(200, 450, ImGuiCond_Always);     // Size of the window
			ImGui::Begin("Choose Test To Run", nullptr, staticWindowFlags);

			vector<string>::const_iterator fileIter = app.files.begin();
			vector<string>::const_iterator filesEnd = app.files.end();
			vector<string>::const_iterator testToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					testToLoad = fileIter;
				}
			}

			if (ImGui::BeginCombo("Dropdown", items[current_item].c_str()))
			{
				for (int i = 0; i < items.size(); i++)
				{
					bool is_selected = (current_item == i);
					if (ImGui::Selectable(items[i].c_str(), is_selected))
					{
						current_item = i;
					}
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			if (testToLoad != filesEnd)
			{
				string path = app.testCasesFolder + "/" + *testToLoad;
				app.testCase = new TestCase;
				if (app.testCase)
				{
					// Load the test.
					if (!app.testCase->load(path))
					{
						delete app.testCase;
						app.testCase = 0;
					}

					// Create sample
					Sample* newSample = 0;
					for (int i = 0; i < g_nsamples; ++i)
					{
						if (g_samples[i].name == app.testCase->getSampleName())
						{
							newSample = g_samples[i].create();
							if (newSample)
							{
								app.sampleIndex = i;
							}
						}
					}

					delete app.sample;
					app.sample = newSample;

					if (app.sample)
					{
						app.sample->setContext(&app.buildContext);
						app.showSample = false;
					}

					// Load geom.
					app.meshName = app.testCase->getGeomFileName();

					path = app.meshesFolder + "/" + app.meshName;

					delete app.inputGeometry;
					app.inputGeometry = new InputGeom;
					if (!app.inputGeometry->load(&app.buildContext, path))
					{
						delete app.inputGeometry;
						app.inputGeometry = nullptr;
						delete app.sample;
						app.sample = nullptr;
						app.showLog = true;
						app.logScroll = 0;
						app.buildContext.dumpLog("geom load log %s:", app.meshName.c_str());
					}
					if (app.sample && app.inputGeometry)
					{
						app.sample->handleMeshChanged(app.inputGeometry);
					}

					// This will ensure that tile & poly bits are updated in tiled sample.
					if (app.sample)
					{
						app.sample->handleSettings();
					}

					app.buildContext.resetLog();
					if (app.sample && !app.sample->handleBuild())
					{
						app.buildContext.dumpLog("Build log %s:", app.meshName.c_str());
					}

					if (app.inputGeometry || app.sample)
					{
						const float* bmin = 0;
						const float* bmax = 0;
						if (app.inputGeometry)
						{
							bmin = app.inputGeometry->getNavMeshBoundsMin();
							bmax = app.inputGeometry->getNavMeshBoundsMax();
						}
						// Reset camera and fog to match the mesh bounds.
						if (bmin && bmax)
						{
							app.camr =
								sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
							app.cameraPos[0] = (bmax[0] + bmin[0]) / 2 + app.camr;
							app.cameraPos[1] = (bmax[1] + bmin[1]) / 2 + app.camr;
							app.cameraPos[2] = (bmax[2] + bmin[2]) / 2 + app.camr;
							app.camr *= 3;
						}
						app.cameraEulers[0] = 45;
						app.cameraEulers[1] = -45;
						glFogf(GL_FOG_START, app.camr * 0.2f);
						glFogf(GL_FOG_END, app.camr * 1.25f);
					}

					// Do the tests.
					if (app.sample)
					{
						app.testCase->doTests(app.sample->getNavMesh(), app.sample->getNavMeshQuery());
					}
				}
			}

			imguiEndScrollArea();
		}
#endif

		// Log
		if (app.showLog && app.showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2(250 + 20, static_cast<float>(app.height - 200 - 10)), ImGuiCond_Always);  // Position in screen space
			ImGui::SetNextWindowSize(ImVec2(static_cast<float>(app.width - 250 - 350 - 20 - 20), 200), ImGuiCond_Always);     // Size of the window
			ImGui::Begin("Log", nullptr, staticWindowFlags);

			for (int i = 0; i < app.buildContext.getLogCount(); ++i)
			{
				ImGui::TextUnformatted(app.buildContext.getLogText(i));
			}

			ImGui::End();
		}

		// Left column tools menu
		if (!app.showTestCases && app.showTools && app.showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);  // Position in screen space
			ImGui::SetNextWindowSize(ImVec2(250, static_cast<float>(app.height - 20)), ImGuiCond_Always);     // Size of the window
			ImGui::Begin("Tools", nullptr, staticWindowFlags);

			if (app.sample)
			{
				app.sample->handleTools();
			}

			ImGui::End();
		}

		// Draw Marker
		if (app.markerPositionSet && gluProject(app.markerPosition[0], app.markerPosition[1], app.markerPosition[2], modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
		{
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240, 220, 0, 196);
			glBegin(GL_LINE_LOOP);
			for (int i = 0; i < 20; ++i)
			{
				constexpr float radius = 25.0f;
				const float arc = static_cast<float>(i) / 20.0f * RC_PI * 2;
				const float vertexX = static_cast<float>(x) + cosf(arc) * radius;
				const float vertexY = static_cast<float>(y) + sinf(arc) * radius;
				glVertex2f(vertexX, vertexY);
			}
			glEnd();
			glLineWidth(1.0f);
		}

		glEnable(GL_DEPTH_TEST);

		ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(app.window);
	}

	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(app.glContext);
    SDL_DestroyWindow(app.window);
    SDL_Quit();

	return 0;
}
