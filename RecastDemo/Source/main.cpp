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
#include "SDL_opengl.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include "Filelist.h"
#include "InputGeom.h"
#include "Recast.h"
#include "Sample_SoloMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_TileMesh.h"
#include "TestCase.h"
#include "imgui.h"
#include "imguiRenderGL.h"

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
static SampleItem g_samples[] = {
	{createSolo, "Solo Mesh"},
	{createTile, "Tile Mesh"},
	{createTempObstacle, "Temp Obstacles"},
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

struct AppData
{
	// Window & SDL
	int width;
	int height;
	SDL_Window* window;
	SDL_Renderer* renderer;

	// Recast and Samples
	InputGeom* inputGeometry = nullptr;
	Sample* sample = nullptr;
	TestCase* testCase = nullptr;

	// Time
	float timeAcc = 0.0f;
	Uint32 prevFrameTime = 0;

	// Input
	int mousePos[2] {0, 0};
	int origMousePos[2] {0, 0};  // Used to compute mouse movement totals across frames.

	// Camera
	float cameraEulers[2] {45, -45};
	float cameraPos[3] = {0, 0, 0};
	float camr = 1000;
	float origCameraEulers[2] = {0, 0};	// Used to compute rotational changes across frames.

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
	string sampleName = "Choose Sample...";
	string meshName = "Choose Mesh...";

	// UI state
	bool showMenu = true;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;

	// Window scroll positions.
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;

	// Files
	vector<string> files;
	const string meshesFolder = "Meshes";
	const string testCasesFolder = "TestCases";

	// Markers
	float markerPosition[3] = {0, 0, 0};
	bool markerPositionSet = false;
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

	app.width = rcMin(displayMode.w, (int)(displayMode.h * (16.0f / 9.0f))) - 80;
	app.height = displayMode.h - 80;

	int errorCode = SDL_CreateWindowAndRenderer(app.width, app.height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &app.window, &app.renderer);

	if (errorCode != 0 || !app.window || !app.renderer)
	{
		printf("Could not initialise SDL opengl\nError: %s\n", SDL_GetError());
		return -1;
	}

	SDL_SetWindowPosition(app.window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		printf("Could not init GUI renderer.\n");
		SDL_Quit();
		return -1;
	}

	app.prevFrameTime = SDL_GetTicks();

	// Fog.
	float fogColor[4] = {0.32f, 0.31f, 0.30f, 1.0f};
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, app.camr * 0.1f);
	glFogf(GL_FOG_END, app.camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);

	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);

	bool done = false;
	while (!done)
	{
		// Handle input events.
		int mouseScroll = 0;
		bool processHitTest = false;
		bool processHitTestShift = false;
		SDL_Event event;

		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
			case SDL_KEYDOWN:
				// Handle any key presses here.
				if (event.key.keysym.sym == SDLK_ESCAPE)
				{
					done = true;
				}
				else if (event.key.keysym.sym == SDLK_t)
				{
					app.showLevels = false;
					app.showSample = false;
					app.showTestCases = true;
					scanDirectory(app.testCasesFolder, ".txt", app.files);
				}
				else if (event.key.keysym.sym == SDLK_TAB)
				{
					app.showMenu = !app.showMenu;
				}
				else if (event.key.keysym.sym == SDLK_SPACE)
				{
					if (app.sample)
					{
						app.sample->handleToggle();
					}
				}
				else if (event.key.keysym.sym == SDLK_1)
				{
					if (app.sample)
					{
						app.sample->handleStep();
					}
				}
				else if (event.key.keysym.sym == SDLK_9)
				{
					if (app.sample && app.inputGeometry)
					{
						string savePath = app.meshesFolder + "/";
						BuildSettings settings;
						memset(&settings, 0, sizeof(settings));

						rcVcopy(settings.navMeshBMin, app.inputGeometry->getNavMeshBoundsMin());
						rcVcopy(settings.navMeshBMax, app.inputGeometry->getNavMeshBoundsMax());

						app.sample->collectSettings(settings);

						app.inputGeometry->saveGeomSet(&settings);
					}
				}
				break;

			case SDL_MOUSEWHEEL:
				if (event.wheel.y < 0)
				{
					// wheel down
					if (app.mouseOverMenu)
					{
						mouseScroll++;
					}
					else
					{
						app.scrollZoom += 1.0f;
					}
				}
				else
				{
					if (app.mouseOverMenu)
					{
						mouseScroll--;
					}
					else
					{
						app.scrollZoom -= 1.0f;
					}
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					if (!app.mouseOverMenu)
					{
						// Rotate view
						app.isRotatingCamera = true;
						app.movedDuringRotate = false;
						app.origMousePos[0] = app.mousePos[0];
						app.origMousePos[1] = app.mousePos[1];
						app.origCameraEulers[0] = app.cameraEulers[0];
						app.origCameraEulers[1] = app.cameraEulers[1];
					}
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
					app.cameraEulers[0] = app.origCameraEulers[0] - dy * 0.25f;
					app.cameraEulers[1] = app.origCameraEulers[1] + dx * 0.25f;
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

		unsigned char mouseButtonMask = 0;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
		{
			mouseButtonMask |= IMGUI_MBUT_LEFT;
		}
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
		{
			mouseButtonMask |= IMGUI_MBUT_RIGHT;
		}

		Uint32 time = SDL_GetTicks();
		float dt = (time - app.prevFrameTime) / 1000.0f;
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
					float pos[3];
					pos[0] = app.rayStart[0] + (app.rayEnd[0] - app.rayStart[0]) * hitTime;
					pos[1] = app.rayStart[1] + (app.rayEnd[1] - app.rayStart[1]) * hitTime;
					pos[2] = app.rayStart[2] + (app.rayEnd[2] - app.rayStart[2]) * hitTime;
					app.sample->handleClick(app.rayStart, pos, processHitTestShift);
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
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f / SIM_RATE;
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
		const float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt) * 1000.0f);
			if (ms > 10)
			{
				ms = 10;
			}
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
		app.moveFront = rcClamp(app.moveFront + dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP]) ? 1 : -1), 0.0f, 1.0f);
		app.moveLeft = rcClamp(app.moveLeft + dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT]) ? 1 : -1), 0.0f, 1.0f);
		app.moveBack = rcClamp(app.moveBack + dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN]) ? 1 : -1), 0.0f, 1.0f);
		app.moveRight = rcClamp(app.moveRight + dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT]) ? 1 : -1), 0.0f, 1.0f);
		app.moveUp = rcClamp(app.moveUp + dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP]) ? 1 : -1), 0.0f, 1.0f);
		app.moveDown = rcClamp(app.moveDown + dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN]) ? 1 : -1), 0.0f, 1.0f);

		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
		{
			keybSpeed *= 4.0f;
		}

		float movex = (app.moveRight - app.moveLeft) * keybSpeed * dt;
		float movey = (app.moveBack - app.moveFront) * keybSpeed * dt + app.scrollZoom * 2.0f;
		app.scrollZoom = 0;

		app.cameraPos[0] += movex * (float)modelviewMatrix[0];
		app.cameraPos[1] += movex * (float)modelviewMatrix[4];
		app.cameraPos[2] += movex * (float)modelviewMatrix[8];

		app.cameraPos[0] += movey * (float)modelviewMatrix[2];
		app.cameraPos[1] += movey * (float)modelviewMatrix[6];
		app.cameraPos[2] += movey * (float)modelviewMatrix[10];

		app.cameraPos[1] += (app.moveUp - app.moveDown) * keybSpeed * dt;

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

		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, app.width, 0, app.height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		app.mouseOverMenu = false;

		imguiBeginFrame(app.mousePos[0], app.mousePos[1], mouseButtonMask, mouseScroll);

		if (app.sample)
		{
			app.sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
		}
		if (app.testCase)
		{
			if (app.testCase->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
			{
				app.mouseOverMenu = true;
			}
		}

		// Help text.
		if (app.showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, app.height - 20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255, 255, 255, 128));
		}

		if (app.showMenu)
		{
			if (imguiBeginScrollArea("Properties", app.width - 250 - 10, 10, 250, app.height - 20, &app.propScroll))
			{
				app.mouseOverMenu = true;
			}

			if (imguiCheck("Show Log", app.showLog))
			{
				app.showLog = !app.showLog;
			}
			if (imguiCheck("Show Tools", app.showTools))
			{
				app.showTools = !app.showTools;
			}

			imguiSeparator();
			imguiLabel("Sample");
			if (imguiButton(app.sampleName.c_str()))
			{
				if (app.showSample)
				{
					app.showSample = false;
				}
				else
				{
					app.showSample = true;
					app.showLevels = false;
					app.showTestCases = false;
				}
			}

			imguiSeparator();
			imguiLabel("Input Mesh");
			if (imguiButton(app.meshName.c_str()))
			{
				if (app.showLevels)
				{
					app.showLevels = false;
				}
				else
				{
					app.showSample = false;
					app.showTestCases = false;
					app.showLevels = true;
					scanDirectory(app.meshesFolder, ".obj", app.files);
					scanDirectoryAppend(app.meshesFolder, ".gset", app.files);
				}
			}
			if (app.inputGeometry)
			{
				char text[64];
				snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk", app.inputGeometry->getMesh()->getVertCount() / 1000.0f, app.inputGeometry->getMesh()->getTriCount() / 1000.0f);
				imguiValue(text);
			}
			imguiSeparator();

			if (app.inputGeometry && app.sample)
			{
				imguiSeparatorLine();

				app.sample->handleSettings();

				if (imguiButton("Build"))
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

				imguiSeparator();
			}

			if (app.sample)
			{
				imguiSeparatorLine();
				app.sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}

		// Sample selection dialog.
		if (app.showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose sample", app.width - 10 - 250 - 10 - 200, app.height - 10 - 250, 200, 250, &levelScroll))
			{
				app.mouseOverMenu = true;
			}

			Sample* newSample = 0;
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name.c_str()))
				{
					newSample = g_samples[i].create();
					if (newSample)
					{
						app.sampleName = g_samples[i].name;
					}
				}
			}
			if (newSample)
			{
				delete app.sample;
				app.sample = newSample;
				app.sample->setContext(&app.buildContext);
				if (app.inputGeometry)
				{
					app.sample->handleMeshChanged(app.inputGeometry);
				}
				app.showSample = false;
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
					app.camr = sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
					app.cameraPos[0] = (bmax[0] + bmin[0]) / 2 + app.camr;
					app.cameraPos[1] = (bmax[1] + bmin[1]) / 2 + app.camr;
					app.cameraPos[2] = (bmax[2] + bmin[2]) / 2 + app.camr;
					app.camr *= 3;
				}
				app.cameraEulers[0] = 45;
				app.cameraEulers[1] = -45;
				glFogf(GL_FOG_START, app.camr * 0.1f);
				glFogf(GL_FOG_END, app.camr * 1.25f);
			}

			imguiEndScrollArea();
		}

		// Level selection dialog.
		if (app.showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Level", app.width - 10 - 250 - 10 - 200, app.height - 10 - 450, 200, 450, &levelScroll))
			{
				app.mouseOverMenu = true;
			}

			vector<string>::const_iterator fileIter = app.files.begin();
			vector<string>::const_iterator filesEnd = app.files.end();
			vector<string>::const_iterator levelToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					levelToLoad = fileIter;
				}
			}

			if (levelToLoad != filesEnd)
			{
				app.meshName = *levelToLoad;
				app.showLevels = false;

				delete app.inputGeometry;
				app.inputGeometry = 0;

				string path = app.meshesFolder + "/" + app.meshName;

				app.inputGeometry = new InputGeom;
				if (!app.inputGeometry->load(&app.buildContext, path))
				{
					delete app.inputGeometry;
					app.inputGeometry = 0;

					// Destroy the sample if it already had geometry loaded, as we've just deleted it!
					if (app.sample && app.sample->getInputGeom())
					{
						delete app.sample;
						app.sample = 0;
					}

					app.showLog = true;
					app.logScroll = 0;
					app.buildContext.dumpLog("geom load log %s:", app.meshName.c_str());
				}
				if (app.sample && app.inputGeometry)
				{
					app.sample->handleMeshChanged(app.inputGeometry);
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
						app.camr = sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
						app.cameraPos[0] = (bmax[0] + bmin[0]) / 2 + app.camr;
						app.cameraPos[1] = (bmax[1] + bmin[1]) / 2 + app.camr;
						app.cameraPos[2] = (bmax[2] + bmin[2]) / 2 + app.camr;
						app.camr *= 3;
					}
					app.cameraEulers[0] = 45;
					app.cameraEulers[1] = -45;
					glFogf(GL_FOG_START, app.camr * 0.1f);
					glFogf(GL_FOG_END, app.camr * 1.25f);
				}
			}

			imguiEndScrollArea();
		}

		// Test cases
		if (app.showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", app.width - 10 - 250 - 10 - 200, app.height - 10 - 450, 200, 450, &testScroll))
			{
				app.mouseOverMenu = true;
			}

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
								app.sampleName = g_samples[i].name;
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
					if (!app.inputGeometry || !app.inputGeometry->load(&app.buildContext, path))
					{
						delete app.inputGeometry;
						app.inputGeometry = 0;
						delete app.sample;
						app.sample = 0;
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
							app.camr = sqrtf(rcSqr(bmax[0] - bmin[0]) + rcSqr(bmax[1] - bmin[1]) + rcSqr(bmax[2] - bmin[2])) / 2;
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

		// Log
		if (app.showLog && app.showMenu)
		{
			if (imguiBeginScrollArea("Log", 250 + 20, 10, app.width - 300 - 250, 200, &app.logScroll))
			{
				app.mouseOverMenu = true;
			}
			for (int i = 0; i < app.buildContext.getLogCount(); ++i)
			{
				imguiLabel(app.buildContext.getLogText(i));
			}
			imguiEndScrollArea();
		}

		// Left column tools menu
		if (!app.showTestCases && app.showTools && app.showMenu)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, app.height - 20, &app.toolsScroll))
			{
				app.mouseOverMenu = true;
			}

			if (app.sample)
			{
				app.sample->handleTools();
			}

			imguiEndScrollArea();
		}

		// Marker
		if (app.markerPositionSet && gluProject((GLdouble)app.markerPosition[0], (GLdouble)app.markerPosition[1], (GLdouble)app.markerPosition[2], modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
		{
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240, 220, 0, 196);
			glBegin(GL_LINE_LOOP);
			const float r = 25.0f;
			for (int i = 0; i < 20; ++i)
			{
				const float a = (float)i / 20.0f * RC_PI * 2;
				const float fx = (float)x + cosf(a) * r;
				const float fy = (float)y + sinf(a) * r;
				glVertex2f(fx, fy);
			}
			glEnd();
			glLineWidth(1.0f);
		}

		imguiEndFrame();
		imguiRenderGLDraw();

		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapWindow(app.window);
	}

	imguiRenderGLDestroy();

	SDL_Quit();

	delete app.sample;
	delete app.inputGeometry;

	return 0;
}
