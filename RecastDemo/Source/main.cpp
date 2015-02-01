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

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "SlideShow.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"

#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "imguiRenderGL.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <iomanip>
#include <cstdlib>

#ifdef WIN32
#	define putenv _putenv
#endif

struct SampleItem
{
	Sample* (*create)();
	const char* name;
};

Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }

static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
	//	{ createDebug, "Debug" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

// Function forward-declares
int run(int width, int height, bool presentationMode);
void drawMarker(float markerPosition[3], GLdouble projectionMatrix[16], GLdouble modelviewMatrix[16], GLint viewport[4]);

int main(int /*argc*/, char** /*argv*/)
{
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		std::cerr << "Could not initialise SDL\n";
		return -1;
	}

	// Center window
	putenv("SDL_VIDEO_CENTERED=1");

	// Init OpenGL
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	bool presentationMode = false;

	int width;
	int height;
	SDL_Surface* screen = 0;

	const SDL_VideoInfo* videoInfo = SDL_GetVideoInfo();
	if (presentationMode)
	{
		width = videoInfo->current_w;
		height = videoInfo->current_h;
		screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL | SDL_FULLSCREEN);
	}
	else
	{
		width = rcMin(videoInfo->current_w, (int)(videoInfo->current_h * 16.0 / 9.0));
		width = width - 80;
		height = videoInfo->current_h - 80;
		screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL);
	}

	if (!screen)
	{
		std::cerr << "Fatal error: Could not initialise SDL opengl!\n";
		return -1;
	}

	glEnable(GL_MULTISAMPLE);

	SDL_WM_SetCaption("Recast Demo", 0);

	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		std::cerr << "Fatal error: Could not init GUI renderer.\n";
		SDL_Quit();
		return -1;
	}

	int retval = run(width, height, presentationMode);
	SDL_Quit();
	return retval;
}

int run(int width, int height, bool presentationMode) {

	float totalTime = 0.0f;
	float timeAcc = 0.0f;
	Uint32 lastTime = SDL_GetTicks();
	int mx = 0;
	int my = 0;
	float rx = 45;
	float ry = -45;

	float moveW = 0;
	float moveS = 0;
	float moveA = 0;
	float moveD = 0;

	float camx = 0;
	float camy = 0;
	float camz = 0;
	float camr = 1000;

	float origrx = 0, origry = 0;
	int origx = 0, origy = 0;
	float scrollZoom = 0;
	bool rotate = false;
	bool movedDuringRotate = false;
	float rays[3], raye[3];
	bool mouseOverMenu = false;
	bool showMenu = !presentationMode;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;

	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;

	string sampleName = "Choose Sample...";

	vector<string> files;
	string meshName = "Choose Mesh...";

	float markerPosition[3] = { 0, 0, 0 };
	bool mposSet = false;

	SlideShow slideShow("slides/");

	std::unique_ptr<InputGeom> geom;
	std::unique_ptr<Sample> sample;
	TestCase* test = 0;

	BuildContext ctx;

	glEnable(GL_CULL_FACE);

	float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);

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
					showLevels = false;
					showSample = false;
					showTestCases = true;
					scanDirectory("Tests", ".txt", files);
				}
				else if (event.key.keysym.sym == SDLK_TAB)
				{
					showMenu = !showMenu;
				}
				else if (event.key.keysym.sym == SDLK_SPACE)
				{
					if (sample)
						sample->handleToggle();
				}
				else if (event.key.keysym.sym == SDLK_1)
				{
					if (sample)
						sample->handleStep();
				}
				else if (event.key.keysym.sym == SDLK_9)
				{
					if (geom)
						geom->save("geomset.txt");
				}
				else if (event.key.keysym.sym == SDLK_0)
				{
					geom = std::unique_ptr<InputGeom>(new InputGeom);
					if (!geom->load(&ctx, "geomset.txt"))
					{
						geom = nullptr;

						showLog = true;
						logScroll = 0;
						ctx.dumpLog("Geom load log %s:", meshName.c_str());
					}
					if (sample && geom)
					{
						sample->handleMeshChanged(geom.get());
					}

					if (geom || sample)
					{
						const float* bmin = 0;
						const float* bmax = 0;
						if (sample)
						{
							bmin = sample->getBoundsMin();
							bmax = sample->getBoundsMax();
						}
						else if (geom)
						{
							bmin = geom->getMeshBoundsMin();
							bmax = geom->getMeshBoundsMax();
						}
						// Reset camera and fog to match the mesh bounds.
						if (bmin && bmax)
						{
							camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
								rcSqr(bmax[1] - bmin[1]) +
								rcSqr(bmax[2] - bmin[2])) / 2;
							camx = (bmax[0] + bmin[0]) / 2 + camr;
							camy = (bmax[1] + bmin[1]) / 2 + camr;
							camz = (bmax[2] + bmin[2]) / 2 + camr;
							camr *= 3;
						}
						rx = 45;
						ry = -45;
						glFogf(GL_FOG_START, camr*0.2f);
						glFogf(GL_FOG_END, camr*1.25f);
					}
				}
				else if (event.key.keysym.sym == SDLK_RIGHT)
				{
					slideShow.nextSlide();
				}
				else if (event.key.keysym.sym == SDLK_LEFT)
				{
					slideShow.prevSlide();
				}
				break;

			case SDL_MOUSEBUTTONDOWN:
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					if (!mouseOverMenu)
					{
						// Rotate view
						rotate = true;
						movedDuringRotate = false;
						origx = mx;
						origy = my;
						origrx = rx;
						origry = ry;
					}
				}
				else if (event.button.button == SDL_BUTTON_WHEELUP)
				{
					if (mouseOverMenu)
						mouseScroll--;
					else
						scrollZoom -= 1.0f;
				}
				else if (event.button.button == SDL_BUTTON_WHEELDOWN)
				{
					if (mouseOverMenu)
						mouseScroll++;
					else
						scrollZoom += 1.0f;
				}
				break;

			case SDL_MOUSEBUTTONUP:
				// Handle mouse clicks here.
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					rotate = false;
					if (!mouseOverMenu)
					{
						if (!movedDuringRotate)
						{
							processHitTest = true;
							processHitTestShift = true;
						}
					}
				}
				else if (event.button.button == SDL_BUTTON_LEFT)
				{
					if (!mouseOverMenu)
					{
						processHitTest = true;
						processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
					}
				}

				break;

			case SDL_MOUSEMOTION:
				mx = event.motion.x;
				my = height - 1 - event.motion.y;
				if (rotate)
				{
					int dx = mx - origx;
					int dy = my - origy;
					rx = origrx - dy*0.25f;
					ry = origry + dx*0.25f;
					if (dx*dx + dy*dy > 3 * 3)
						movedDuringRotate = true;
				}
				break;

			case SDL_QUIT:
				done = true;
				break;

			default:
				break;
			}
		}

		unsigned char mouseButton = 0;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
			mouseButton |= IMGUI_MBUT_LEFT;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
			mouseButton |= IMGUI_MBUT_RIGHT;

		Uint32	time = SDL_GetTicks();
		float	dt = (time - lastTime) / 1000.0f;
		lastTime = time;

		totalTime += dt;

		// Hit test mesh.
		if (processHitTest && geom && sample)
		{
			float hitTime;
			bool hit = geom->raycastMesh(rays, raye, hitTime);

			if (hit)
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					mposSet = true;
					markerPosition[0] = rays[0] + (raye[0] - rays[0]) * hitTime;
					markerPosition[1] = rays[1] + (raye[1] - rays[1]) * hitTime;
					markerPosition[2] = rays[2] + (raye[2] - rays[2]) * hitTime;
				}
				else
				{
					float pos[3];
					pos[0] = rays[0] + (raye[0] - rays[0]) * hitTime;
					pos[1] = rays[1] + (raye[1] - rays[1]) * hitTime;
					pos[2] = rays[2] + (raye[2] - rays[2]) * hitTime;
					sample->handleClick(rays, pos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					mposSet = false;
				}
			}
		}

		// Update sample simulation.
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f / SIM_RATE;
		timeAcc = rcClamp(timeAcc + dt, -1.0f, 1.0f);
		int simIter = 0;
		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;
			if (simIter < 5)
			{
				if (sample)
					sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		const float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt)*1000.0f);
			if (ms > 10) ms = 10;
			if (ms >= 0)
				SDL_Delay(ms);
		}


		// Update and render
		glViewport(0, 0, width, height);
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);

		// Render 3d
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width / (float)height, 1.0f, camr);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(rx, 1, 0, 0);
		glRotatef(ry, 0, 1, 0);
		glTranslatef(-camx, -camy, -camz);

		// Get hit ray position and direction.
		GLdouble projectionMatrix[16];
		GLdouble modelviewMatrix[16];
		GLint viewport[4];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
		glGetIntegerv(GL_VIEWPORT, viewport);
		GLdouble x, y, z;
		gluUnProject(mx, my, 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rays[0] = (float)x; rays[1] = (float)y; rays[2] = (float)z;
		gluUnProject(mx, my, 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		raye[0] = (float)x; raye[1] = (float)y; raye[2] = (float)z;

		// Handle keyboard movement.
		Uint8* keystate = SDL_GetKeyState(NULL);
		moveW = rcClamp(moveW + dt * 4 * (keystate[SDLK_w] ? 1 : -1), 0.0f, 1.0f);
		moveS = rcClamp(moveS + dt * 4 * (keystate[SDLK_s] ? 1 : -1), 0.0f, 1.0f);
		moveA = rcClamp(moveA + dt * 4 * (keystate[SDLK_a] ? 1 : -1), 0.0f, 1.0f);
		moveD = rcClamp(moveD + dt * 4 * (keystate[SDLK_d] ? 1 : -1), 0.0f, 1.0f);

		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
			keybSpeed *= 4.0f;

		float movex = (moveD - moveA) * keybSpeed * dt;
		float movey = (moveS - moveW) * keybSpeed * dt;

		movey += scrollZoom * 2.0f;
		scrollZoom = 0;

		camx += movex * (float)modelviewMatrix[0];
		camy += movex * (float)modelviewMatrix[4];
		camz += movex * (float)modelviewMatrix[8];

		camx += movey * (float)modelviewMatrix[2];
		camy += movey * (float)modelviewMatrix[6];
		camz += movey * (float)modelviewMatrix[10];

		glEnable(GL_FOG);

		if (sample)
			sample->handleRender();
		if (test)
			test->handleRender();

		glDisable(GL_FOG);

		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		mouseOverMenu = false;

		imguiBeginFrame(mx, my, mouseButton, mouseScroll);

		if (sample)
		{
			sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
		}
		if (test)
		{
			if (test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
				mouseOverMenu = true;
		}

		// Help text.
		if (showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, height - 20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255, 255, 255, 128));
		}

		if (showMenu)
		{
			if (imguiBeginScrollArea("Properties", width - 250 - 10, 10, 250, height - 20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("Show Log", showLog))
				showLog = !showLog;
			if (imguiCheck("Show Tools", showTools))
				showTools = !showTools;

			imguiSeparator();
			imguiLabel("Sample");
			if (imguiButton(sampleName.c_str()))
			{
				if (showSample)
				{
					showSample = false;
				}
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
				}
			}

			imguiSeparator();
			imguiLabel("Input Mesh");
			if (imguiButton(meshName.c_str()))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;
					scanDirectory("Meshes", ".obj", files);
				}
			}
			if (geom)
			{
				float kVerts = geom->getMesh()->getVertCount() / 1000.0f;
				float kTris = geom->getMesh()->getTriCount() / 1000.0f;
				std::stringstream text;
				text << std::fixed << std::setprecision(1);
				text << "Verts: " << kVerts << "k  Tris: " << kTris << "k";
				imguiValue(text.str().c_str());
			}
			imguiSeparator();

			if (geom && sample)
			{
				imguiSeparatorLine();

				sample->handleSettings();

				if (imguiButton("Build"))
				{
					ctx.resetLog();
					if (!sample->handleBuild())
					{
						showLog = true;
						logScroll = 0;
					}
					ctx.dumpLog("Build log %s:", meshName.c_str());

					// Clear test.
					delete test;
					test = 0;
				}

				imguiSeparator();
			}

			if (sample)
			{
				imguiSeparatorLine();
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}

		// Sample selection dialog.
		if (showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Sample", width - 10 - 250 - 10 - 200, height - 10 - 250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name))
				{
					sample = std::unique_ptr<Sample>(g_samples[i].create());
					sample->setContext(&ctx);
					sampleName = g_samples[i].name;
					if (geom && sample)
					{
						sample->handleMeshChanged(geom.get());
					}
					showSample = false;
				}
			}

			if (geom || sample)
			{
				const float* bmin = 0;
				const float* bmax = 0;
				if (sample)
				{
					bmin = sample->getBoundsMin();
					bmax = sample->getBoundsMax();
				}
				else if (geom)
				{
					bmin = geom->getMeshBoundsMin();
					bmax = geom->getMeshBoundsMax();
				}
				// Reset camera and fog to match the mesh bounds.
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
						rcSqr(bmax[1] - bmin[1]) +
						rcSqr(bmax[2] - bmin[2])) / 2;
					camx = (bmax[0] + bmin[0]) / 2 + camr;
					camy = (bmax[1] + bmin[1]) / 2 + camr;
					camz = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}
				rx = 45;
				ry = -45;
				glFogf(GL_FOG_START, camr*0.1f);
				glFogf(GL_FOG_END, camr*1.25f);
			}

			imguiEndScrollArea();
		}

		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
				mouseOverMenu = true;

			string levelName = "";

			for (string file : files)
				if (imguiItem(file.c_str()))
					levelName = file;

			if (levelName != "")
			{
				meshName = levelName;
				showLevels = false;

				string path = "Meshes/";
				path += meshName;

				geom = std::unique_ptr<InputGeom>(new InputGeom);
				if (!geom->loadMesh(&ctx, path.c_str()))
				{
					geom = nullptr;

					showLog = true;
					logScroll = 0;
					ctx.dumpLog("Geom load log %s:", meshName.c_str());
				}
				if (sample && geom)
				{
					sample->handleMeshChanged(geom.get());
				}

				if (geom || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (sample)
					{
						bmin = sample->getBoundsMin();
						bmax = sample->getBoundsMax();
					}
					else if (geom)
					{
						bmin = geom->getMeshBoundsMin();
						bmax = geom->getMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
							rcSqr(bmax[1] - bmin[1]) +
							rcSqr(bmax[2] - bmin[2])) / 2;
						camx = (bmax[0] + bmin[0]) / 2 + camr;
						camy = (bmax[1] + bmin[1]) / 2 + camr;
						camz = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					rx = 45;
					ry = -45;
					glFogf(GL_FOG_START, camr*0.1f);
					glFogf(GL_FOG_END, camr*1.25f);
				}
			}

			imguiEndScrollArea();

		}

		// Test cases
		if (showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &testScroll))
				mouseOverMenu = true;

			// Get the name of the currently selected test.
			string testName = "";
			for (string testfilename : files)
				if (imguiItem(testfilename.c_str()))
					testName = testfilename;

			if (testName != "")
			{
				string path = "Tests/" + testName;
				test = new TestCase;

				// Load the test.
				if (!test->load(path))
				{
					delete test;
					test = nullptr;
				}

				// Create sample
				std::unique_ptr<Sample> newSample;
				for (int i = 0; i < g_nsamples; ++i)
				{
					if (strcmp(g_samples[i].name, test->getSampleName().c_str()) == 0)
					{
						newSample = std::unique_ptr<Sample>(g_samples[i].create());
						if (newSample)
							sampleName = g_samples[i].name;
					}
				}
				if (newSample)
				{
					sample = std::move(newSample);
					sample->setContext(&ctx);
					showSample = false;
				}

				// Load geom.
				meshName = test->getGeomFileName();

				path = "Meshes/";
				path += meshName;

				geom = std::unique_ptr<InputGeom>(new InputGeom);
				if (!geom->loadMesh(&ctx, path.c_str()))
				{
					geom = nullptr;
					showLog = true;
					logScroll = 0;
					ctx.dumpLog("Geom load log %s:", meshName.c_str());
				}
				if (sample && geom)
				{
					sample->handleMeshChanged(geom.get());
				}

				// This will ensure that tile & poly bits are updated in tiled sample.
				if (sample)
				{
					sample->handleSettings();
				}

				ctx.resetLog();
				if (sample && !sample->handleBuild())
				{
					ctx.dumpLog("Build log %s:", meshName.c_str());
				}

				if (geom || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (sample)
					{
						bmin = sample->getBoundsMin();
						bmax = sample->getBoundsMax();
					}
					else if (geom)
					{
						bmin = geom->getMeshBoundsMin();
						bmax = geom->getMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
							rcSqr(bmax[1] - bmin[1]) +
							rcSqr(bmax[2] - bmin[2])) / 2;
						camx = (bmax[0] + bmin[0]) / 2 + camr;
						camy = (bmax[1] + bmin[1]) / 2 + camr;
						camz = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					rx = 45;
					ry = -45;
					glFogf(GL_FOG_START, camr*0.2f);
					glFogf(GL_FOG_END, camr*1.25f);
				}

				// Do the tests.
				if (sample)
					test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
			}

			imguiEndScrollArea();
		}


		// Log
		if (showLog && showMenu)
		{
			if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < ctx.getLogCount(); ++i)
				imguiLabel(ctx.getLogText(i));
			imguiEndScrollArea();
		}

		// Tools
		if (!showTestCases && showTools && showMenu) // && geom && sample)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
				mouseOverMenu = true;

			if (sample)
				sample->handleTools();

			imguiEndScrollArea();
		}

		slideShow.updateAndDraw(dt, (float)width, (float)height);

		// Marker
		if (mposSet)
		{
			drawMarker(markerPosition, projectionMatrix, modelviewMatrix, viewport);
		}

		imguiEndFrame();
		imguiRenderGLDraw();

		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapBuffers();
	}

	imguiRenderGLDestroy();

	SDL_Quit();

	return 0;
}

void drawMarker(float markerPosition[3], GLdouble projectionMatrix[16], GLdouble modelviewMatrix[16], GLint viewport[4])
{
	GLdouble windowCoords[3];
	if (gluProject((GLdouble)markerPosition[0], (GLdouble)markerPosition[1], (GLdouble)markerPosition[2],
		modelviewMatrix, projectionMatrix, viewport, &windowCoords[0], &windowCoords[1], &windowCoords[2]))
	{
		// Draw marker circle
		glLineWidth(5.0f);
		glColor4ub(240, 220, 0, 196);
		glBegin(GL_LINE_LOOP);
		const float radius = 25.0f;
		for (int i = 0; i < 20; ++i)
		{
			const float angle = (float)i / 20.0f * RC_PI * 2;
			const float fx = (float)windowCoords[0] + cosf(angle) * radius;
			const float fy = (float)windowCoords[1] + sinf(angle) * radius;
			glVertex2f(fx, fy);
		}
		glEnd();
		glLineWidth(1.0f);
	}
}
