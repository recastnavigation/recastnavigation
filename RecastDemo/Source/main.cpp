#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#endif

#include "SDL.h"
#include "SDL_Opengl.h"

#include "GLFont.h"
#include "imgui.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "MeshLoaderObj.h"
#include "BuilderStatMeshSimple.h"
#include "BuilderStatMeshTiling.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

GLFont g_font;

void drawText(int x, int y, int dir, const char* text, unsigned int col)
{
	if (dir < 0)
		g_font.drawText((float)x - g_font.getTextLength(text), (float)y, text, col);
	else
		g_font.drawText((float)x, (float)y, text, col);
}


struct FileList
{
	static const int MAX_FILES = 256;
	inline FileList() : size(0) {}
	inline ~FileList()
	{
		clear();
	}
	
	void clear()
	{
		for (int i = 0; i < size; ++i)
			delete [] files[i];
		size = 0;
	}
	
	void add(const char* path)
	{
		if (size >= MAX_FILES)
			return;
		int n = strlen(path);
		files[size] = new char[n+1];
		strcpy(files[size], path);
		size++;
	}
	
	static int cmp(const void* a, const void* b)
	{
		return strcmp(*(const char**)a, *(const char**)b);
	}
	
	void sort()
	{
		if (size > 1)
			qsort(files, size, sizeof(char*), cmp);
	}
	
	char* files[MAX_FILES];
	int size;
};

static void scanDirectory(const char* path, const char* ext, FileList& list)
{
	list.clear();
	
#ifdef WIN32
	_finddata_t dir;
	char pathWithExt[MAX_PATH];
	long fh;
	strcpy(pathWithExt, path);
	strcat(pathWithExt, "/*");
	strcat(pathWithExt, ext);
	fh = _findfirst(pathWithExt, &dir);
	if (fh == -1L)
		return;
	do
	{
		list.add(dir.name);
	}
	while (_findnext(fh, &dir) == 0);
	_findclose(fh);
#else
	dirent* current = 0;
	DIR* dp = opendir(path);
	if (!dp)
		return;
	
	while ((current = readdir(dp)) != 0)
	{
		int len = strlen(current->d_name);
		if (len > 4 && strncmp(current->d_name+len-4, ext, 4) == 0)
		{
			list.add(current->d_name);
		}
	}
	closedir(dp);
#endif
	list.sort();
}

bool intersectSegmentTriangle(const float* sp, const float* sq,
							  const float* a, const float* b, const float* c,
							  float &t)
{
	float v, w;
	float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	vsub(ab, b, a);
	vsub(ac, c, a);
	vsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	vcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = vdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	vsub(ap, sp, a);
	t = vdot(ap, norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	vcross(e, qp, ap);
	v = vdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -vdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}

static bool raycast(rcMeshLoaderObj& mesh, float* src, float* dst, float& tmin)
{
	float dir[3];
	vsub(dir, dst, src);
	
	int nt = mesh.getTriCount();
	const float* verts = mesh.getVerts();
	const float* normals = mesh.getNormals();
	const int* tris = mesh.getTris();
	tmin = 1.0f;
	bool hit = false;
	
	for (int i = 0; i < nt*3; i += 3)
	{
		const float* n = &normals[i];
		if (vdot(dir, n) > 0)
			continue;
		
		float t = 1;
		if (intersectSegmentTriangle(src, dst,
									 &verts[tris[i]*3],
									 &verts[tris[i+1]*3],
									 &verts[tris[i+2]*3], t))
		{
			if (t < tmin)
				tmin = t;
			hit = true;
		}
	}
	
	return hit;
}

int main(int argc, char *argv[])
{
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL\n");
		return -1;
	}
	
	// Init OpenGL
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	
	int width = 1200;
	int height = 700;
	SDL_Surface* screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL);
	if (!screen)
	{
		printf("Could not initialise SDL opengl\n");
		return -1;
	}
	
	SDL_WM_SetCaption("Recast Demo", 0);
	
	if(!g_font.create("font.cfnt"))
	{
		printf("Could not load font.\n");
		SDL_Quit();
		return -1;
	}
	
	float t = 0.0f;
	Uint32 lastTime = SDL_GetTicks();
	int mx = 0, my = 0;
	float rx = 45;
	float ry = -45;
	float moveW = 0, moveS = 0, moveA = 0, moveD = 0;
	float camx = 0, camy = 0, camz = 0, camr=10;
	float origrx, origry;
	int origx, origy;
	bool rotate = false;
	float rays[3], raye[3]; 
	bool mouseOverMenu = false;
	bool showLog = false;
	bool showDebugMode = true;
	bool showTools = true;
	bool showLevels = false;
	bool showBuilder = false;

	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;
	int debugScroll = 0;
	
	char builderName[64] = "Choose Builder..."; 
	
	FileList meshFiles;
	char meshName[128] = "Choose Mesh...";
	
	rcMeshLoaderObj* mesh = 0;
	float meshBMin[3], meshBMax[3];
	
	Builder* builder = 0;

	rcLog log;
	log.clear();
	rcSetLog(&log);
	
	glEnable(GL_CULL_FACE);
	
	float fogCol[4] = { 0.32f,0.25f,0.25f,1 };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, 0);
	glFogf(GL_FOG_END, 10);
	glFogfv(GL_FOG_COLOR, fogCol);
	
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		unsigned char mbut = 0;
		SDL_Event event;
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if (event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					break;
					
				case SDL_MOUSEBUTTONDOWN:
					// Handle mouse clicks here.
					if (!mouseOverMenu)
					{
						if (event.button.button == SDL_BUTTON_RIGHT)
						{
							// Rotate view
							rotate = true;
							origx = mx;
							origy = my;
							origrx = rx;
							origry = ry;
						}
						else if (event.button.button == SDL_BUTTON_LEFT)
						{
							// Hit test mesh.
							if (mesh && builder)
							{
								// Hit test mesh.
								float t;
								if (raycast(*mesh, rays, raye, t))
								{
									float pos[3];
									pos[0] = rays[0] + (raye[0] - rays[0])*t;
									pos[1] = rays[1] + (raye[1] - rays[1])*t;
									pos[2] = rays[2] + (raye[2] - rays[2])*t;
									if (SDL_GetModState() & KMOD_SHIFT)
										builder->setToolStartPos(pos);
									else
										builder->setToolEndPos(pos);
								}
							}
						}
					}	
					if (event.button.button == SDL_BUTTON_WHEELUP)
						mbut |= IMGUI_MBUT_UP;
					if (event.button.button == SDL_BUTTON_WHEELDOWN)
						mbut |= IMGUI_MBUT_DOWN;
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if(event.button.button == SDL_BUTTON_RIGHT)
					{
						rotate = false;
					}
					break;
					
				case SDL_MOUSEMOTION:
					mx = event.motion.x;
					my = height-1 - event.motion.y;
					if (rotate)
					{
						int dx = mx - origx;
						int dy = my - origy;
						rx = origrx - dy*0.25f;
						ry = origry + dx*0.25f;
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}

		if (SDL_GetMouseState(0,0) & SDL_BUTTON_LMASK)
			mbut |= IMGUI_MBUT_LEFT;
		if (SDL_GetMouseState(0,0) & SDL_BUTTON_RMASK)
			mbut |= IMGUI_MBUT_RIGHT;
		
		Uint32	time = SDL_GetTicks();
		float	dt = (time - lastTime) / 1000.0f;
		lastTime = time;
		
		t += dt;
		
		// Update and render
		glViewport(0, 0, width, height);
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		
		// Render 3d
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(rx,1,0,0);
		glRotatef(ry,0,1,0);
		glTranslatef(-camx, -camy, -camz);
		
		// Get hit ray position and direction.
		GLdouble proj[16];
		GLdouble model[16];
		GLint view[4];
		glGetDoublev(GL_PROJECTION_MATRIX, proj);
		glGetDoublev(GL_MODELVIEW_MATRIX, model);
		glGetIntegerv(GL_VIEWPORT, view);
		GLdouble x, y, z;
		gluUnProject(mx, my, 0.0f, model, proj, view, &x, &y, &z);
		rays[0] = (float)x; rays[1] = (float)y; rays[2] = (float)z;
		gluUnProject(mx, my, 1.0f, model, proj, view, &x, &y, &z);
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
		
		camx += movex * (float)model[0];
		camy += movex * (float)model[4];
		camz += movex * (float)model[8];
		
		camx += movey * (float)model[2];
		camy += movey * (float)model[6];
		camz += movey * (float)model[10];

		glEnable(GL_FOG);

		if (builder)
			builder->handleRender();

		glDisable(GL_FOG);
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		if (builder)
		{
			builder->handleRenderOverlay(&g_font, (double*)proj, (double*)model, (int*)view);
			glDisable(GL_TEXTURE_2D);
		}
		
		imguiBeginFrame(mx,my,mbut);
		
		mouseOverMenu = false;
		
		int propDiv = showDebugMode ? (int)(height*0.6f) : height;
		
		if (imguiBeginScrollArea(GENID, "Properties",
								 width-250-10, 10+height-propDiv, 250, propDiv-20, &propScroll))
			mouseOverMenu = true;

		if (imguiCheck(GENID, "Show Log", showLog))
			showLog = !showLog;
		if (imguiCheck(GENID, "Show Tools", showTools))
			showTools = !showTools;
		if (imguiCheck(GENID, "Show Debug Mode", showDebugMode))
			showDebugMode = !showDebugMode;

		imguiSeparator();
		imguiLabel(GENID, "Builder");
		if (imguiButton(GENID, builderName))
		{
			if (showBuilder)
			{
				showBuilder = false;
			}
			else
			{
				showBuilder = true;
				showLevels = false;
			}
		}
		
		if (builder)
		{
			imguiSeparator();
			imguiLabel(GENID, "Input Mesh");
			if (imguiButton(GENID, meshName))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showBuilder = false;
					showLevels = true;
					scanDirectory("meshes", ".obj", meshFiles);
				}
			}
			if (mesh)
			{
				char text[64];
				snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk", mesh->getVertCount()/1000.0f, mesh->getTriCount()/1000.0f);
				imguiValue(GENID, text);
			}
			imguiSeparator();
		}
				
		if (mesh && builder)
		{
			builder->handleSettings();

			if (imguiButton(GENID, "Build"))
			{
				log.clear();
				if (!builder->handleBuild())
				{
					showLog = true;
					logScroll = 0;
				}
			}

			imguiSeparator();
		}

		
		imguiEndScrollArea();
		
		if (showDebugMode)
		{
			if (imguiBeginScrollArea(GENID, "Debug Mode",
									 width-250-10, 10,
									 250, height-propDiv-10, &debugScroll))
				mouseOverMenu = true;

			if (builder)
				builder->handleDebugMode();

			imguiEndScrollArea();
		}
		
		// Builder selection dialog.
		if (showBuilder)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea(GENID, "Choose Level", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			Builder* newBuilder = 0;
			if (imguiItem(GENID, "Simple Static Mesh"))
			{
				newBuilder = new BuilderStatMeshSimple();
				if (newBuilder) strcpy(builderName, "Simple Static Mesh");
			}
			if (imguiItem(GENID, "Tiling Static Mesh"))
			{
				newBuilder = new BuilderStatMeshTiling();
				if (newBuilder) strcpy(builderName, "Tiling Static Mesh");
			}
			
			if (newBuilder)
			{
				delete builder;
				builder = newBuilder;
				if (mesh && builder)
				{
					builder->handleMeshChanged(mesh->getVerts(), mesh->getVertCount(),
											   mesh->getTris(), mesh->getNormals(), mesh->getTriCount(),
											   meshBMin, meshBMax);
				}
				showBuilder = false;
			}

			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea(GENID, "Choose Level", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
				mouseOverMenu = true;
			
			int levelToLoad = -1;
			for (int i = 0; i < meshFiles.size; ++i)
			{
				if (imguiItem(GENID1(i), meshFiles.files[i]))
					levelToLoad = i;
			}
			
			if (levelToLoad != -1)
			{
				strncpy(meshName, meshFiles.files[levelToLoad], sizeof(meshName));
				meshName[sizeof(meshName)-1] = '\0';
				showLevels = false;
				
				delete mesh;
				mesh = 0;
				
				char path[256];
				strcpy(path, "meshes/");
				strcat(path, meshName);
				
				mesh = new rcMeshLoaderObj;
				if (!mesh || !mesh->load(path))
				{
					delete mesh;
					mesh = 0;
				}
				
				if (mesh)
					rcCalcBounds(mesh->getVerts(), mesh->getVertCount(), meshBMin, meshBMax);
				
				if (builder)
				{
					builder->handleMeshChanged(mesh->getVerts(), mesh->getVertCount(),
											   mesh->getTris(), mesh->getNormals(), mesh->getTriCount(),
											   meshBMin, meshBMax);
				}
								
				// Reset camera and fog to match the mesh bounds.
				camr = sqrtf(rcSqr(meshBMax[0]-meshBMin[0]) +
				rcSqr(meshBMax[1]-meshBMin[1]) +
				rcSqr(meshBMax[2]-meshBMin[2])) / 2;
				camx = (meshBMax[0] + meshBMin[0]) / 2 + camr;
				camy = (meshBMax[1] + meshBMin[1]) / 2 + camr;
				camz = (meshBMax[2] + meshBMin[2]) / 2 + camr;
				camr *= 3;
				rx = 45;
				ry = -45;
				glFogf(GL_FOG_START, camr*0.2f);
				glFogf(GL_FOG_END, camr*1.25f);
			}
			
			imguiEndScrollArea();
			
		}
		
		// Log
		if (showLog)
		{
			if (imguiBeginScrollArea(GENID, "Log", 10, 10, width - 300, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < log.getMessageCount(); ++i)
				imguiLabel(GENID1(i), log.getMessageText(i));
			imguiEndScrollArea();
		}
		
		// Tools
		if (showTools && mesh && builder)
		{
			if (imguiBeginScrollArea(GENID, "Tools", 10, height - 10 - 200, 150, 200, &toolsScroll))
				mouseOverMenu = true;

			builder->handleTools();
			
			imguiEndScrollArea();
		}
		
		// Help text.
		const char msg[] = "W/S/A/D: Move  RMB: Rotate   LMB: Place Start   LMB+SHIFT: Place End";
		const float len = g_font.getTextLength(msg);
		g_font.drawText(width/2-len/2, (float)height-20.0f, msg, GLFont::RGBA(255,255,255,128));
				
		glDisable(GL_TEXTURE_2D);
		
		imguiEndFrame();
		imguiRender(&drawText);		
		
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapBuffers();
	}
	
	SDL_Quit();
	
	delete builder;
	delete mesh;
	
	return 0;
}
