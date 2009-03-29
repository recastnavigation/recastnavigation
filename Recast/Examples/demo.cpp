//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#endif
#include "SDL.h"
#include "SDL_Opengl.h"
#include "GLFont.h"
#include "RecastTimer.h"
#include "MeshLoaderObj.h"
#include "Recast.h"
#include "RecastLog.h"
#include "RecastDebugDraw.h"
#include "imgui.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

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
	
	char* files[MAX_FILES];
	int size;
};

void scanDirectory(const char* path, const char* ext, FileList& list)
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
}


enum DrawMode
{
	DRAWMODE_POLYMESH,
	DRAWMODE_POLYMESH_TRANS,
	DRAWMODE_MESH,
	DRAWMODE_VOXELS,
	DRAWMODE_VOXELS_WALKABLE,
	DRAWMODE_COMPACT,
	DRAWMODE_COMPACT_DISTANCE,
	DRAWMODE_COMPACT_REGIONS,
	DRAWMODE_RAW_CONTOURS,
	DRAWMODE_CONTOURS,
	MAX_DRAWMODE,
};


GLFont g_font;

void drawText(int x, int y, int dir, const char* text, unsigned int col)
{
	if (dir < 0)
		g_font.drawText((float)x - g_font.getTextLength(text), (float)y, text, col);
	else
		g_font.drawText((float)x, (float)y, text, col);
}


rcMeshLoaderObj* g_mesh = 0;
unsigned char* g_triangleFlags = 0;
rcHeightfield* g_solid = 0;
rcCompactHeightfield* g_chf = 0;
rcContourSet* g_cset = 0;
rcPolyMesh* g_polyMesh = 0;
rcConfig g_cfg;
rcLog g_log;

static bool buildNavigation()
{
	delete g_solid;
	delete g_chf;
	delete g_cset;
	delete g_polyMesh;
	delete [] g_triangleFlags;
	g_solid = 0;
	g_chf = 0;
	g_cset = 0;
	g_polyMesh = 0;
	g_triangleFlags = 0;
	
	g_log.clear();
	rcSetLog(&g_log);
	
	if (!g_mesh)
	{
		g_log.log(RC_LOG_ERROR, "Input mesh is not valid.");
		return false;
	}
	
	
	rcTimeVal startTime = rcGetPerformanceTimer();
	
	rcCalcBounds(g_mesh->getVerts(), g_mesh->getVertCount(), g_cfg.bmin, g_cfg.bmax);
	rcCalcGridSize(g_cfg.bmin, g_cfg.bmax, g_cfg.cs, &g_cfg.width, &g_cfg.height);
	
	g_log.log(RC_LOG_PROGRESS, "Building navigation");
	g_log.log(RC_LOG_PROGRESS, " - %d x %d", g_cfg.width, g_cfg.height);
	g_log.log(RC_LOG_PROGRESS, " - %d verts, %d tris", g_mesh->getVertCount(), g_mesh->getTriCount());
	
	g_triangleFlags = new unsigned char[g_mesh->getTriCount()];
	memset(g_triangleFlags, 0, g_mesh->getTriCount());
	rcMarkWalkableTriangles(g_cfg.walkableSlopeAngle,
							g_mesh->getTris(), g_mesh->getNormals(), g_mesh->getTriCount(),
							g_triangleFlags);
	
	g_solid = new rcHeightfield;
	g_chf = new rcCompactHeightfield;
	g_cset = new rcContourSet;
	g_polyMesh = new rcPolyMesh;
	
	if (!rcBuildNavMesh(g_cfg, g_mesh->getVerts(), g_mesh->getVertCount(),
						g_mesh->getTris(), g_triangleFlags, g_mesh->getTriCount(),
						*g_solid, *g_chf, *g_cset, *g_polyMesh))
	{
		g_log.log(RC_LOG_ERROR, "Could not build navmesh.");
		return false;
	}
	
	rcTimeVal endTime = rcGetPerformanceTimer();
	g_log.log(RC_LOG_PROGRESS, "Build time: %.1f ms", rcGetDeltaTimeUsec(startTime, endTime)/1000.0f);
	g_log.log(RC_LOG_PROGRESS, "NavMesh");
	g_log.log(RC_LOG_PROGRESS, " - %d verts, %d polys", g_polyMesh->nverts, g_polyMesh->npolys);	
	const int navMeshDataSize = g_polyMesh->nverts*3*sizeof(unsigned short) +
	g_polyMesh->npolys*g_polyMesh->nvp*2*sizeof(unsigned short);
	g_log.log(RC_LOG_PROGRESS, " - Approx data size %.1f kB", (float)navMeshDataSize/1024.f);	
	
	for (int i = 0; i < g_log.getMessageCount(); ++i)
	{
		printf("%s\n", g_log.getMessageText(i));
	}
	
	return true;
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
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 32);
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
		printf("Could not load font\n");
		SDL_Quit();
		return -1;
	}
	
	float cellSize = 0.3f;
	float cellHeight = 0.2f;
	float agentHeight = 2.0f;
	float agentRadius = 0.3f;
	float agentMaxClimb = 0.9f;
	float agentMaxSlope = 45.0f;
	float regionMinSize = 50;
	float regionMergeSize = 20;
	float edgeMaxLen = 12.0f;
	float edgeMaxError = 1.5f;
	float vertsPerPoly = 6.0f;
	int drawMode = DRAWMODE_POLYMESH;
	bool showLevels = false;
	bool showLog = false;
	char curLevel[256] = "Choose Level...";
	bool mouseOverMenu = false;
	FileList fileList;
	
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
	float spos[3] = {0,0,0};
	
	glEnable(GL_CULL_FACE);
	
	float fogCol[4] = { 0.1f,0.12f,0.14f,1 };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, 0);
	glFogf(GL_FOG_END, 10);
	glFogfv(GL_FOG_COLOR, fogCol);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		SDL_Event event;
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if(event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					break;

				case SDL_MOUSEBUTTONDOWN:
					// Handle mouse clicks here.
					if (!mouseOverMenu)
					{
						if (event.button.button == SDL_BUTTON_LEFT)
						{
							// Rotate view
							rotate = true;
							origx = mx;
							origy = my;
							origrx = rx;
							origry = ry;
						}
						else if (event.button.button == SDL_BUTTON_RIGHT)
						{
							// Hit test mesh.
							if (g_mesh)
							{
								float t;
								if (raycast(*g_mesh, rays, raye, t))
								{
									spos[0] = rays[0] + (raye[0] - rays[0])*t;
									spos[1] = rays[1] + (raye[1] - rays[1])*t;
									spos[2] = rays[2] + (raye[2] - rays[2])*t;
								}
							}
						}
					}	
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if(event.button.button == SDL_BUTTON_LEFT)
					{
						rotate = false;
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
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}
		
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
		
		if (drawMode == DRAWMODE_MESH)
		{
			if (g_mesh)
				rcDebugDrawMesh(*g_mesh, g_triangleFlags);
		}
		else if (drawMode != DRAWMODE_POLYMESH_TRANS)
		{
			if (g_mesh)
				rcDebugDrawMesh(*g_mesh, 0);
		}
		
		glDepthMask(GL_FALSE);
		
		if (drawMode == DRAWMODE_POLYMESH || drawMode == DRAWMODE_POLYMESH_TRANS)
		{
			if (g_polyMesh)
				rcDebugDrawPolyMesh(*g_polyMesh, g_cfg.bmin, g_cfg.cs, g_cfg.ch);
		}
		
		glDepthMask(GL_TRUE);
		
		if (drawMode == DRAWMODE_COMPACT)
		{
			if (g_chf)
				rcDebugDrawCompactHeightfieldSolid(*g_chf);
		}
		if (drawMode == DRAWMODE_COMPACT_DISTANCE)
		{
			if (g_chf)
				rcDebugDrawCompactHeightfieldDistance(*g_chf);
		}
		if (drawMode == DRAWMODE_COMPACT_REGIONS)
		{
			if (g_chf)
				rcDebugDrawCompactHeightfieldRegions(*g_chf);
		}
		if (drawMode == DRAWMODE_VOXELS)
		{
			if (g_solid)
				rcDebugDrawHeightfieldSolid(*g_solid, g_cfg.bmin, g_cfg.cs, g_cfg.ch);
		}
		if (drawMode == DRAWMODE_VOXELS_WALKABLE)
		{
			if (g_solid)
				rcDebugDrawHeightfieldWalkable(*g_solid, g_cfg.bmin, g_cfg.cs, g_cfg.ch);
		}
		if (drawMode == DRAWMODE_RAW_CONTOURS)
		{
			if (g_cset)
				rcDebugDrawRawContours(*g_cset, g_cfg.bmin, g_cfg.cs, g_cfg.ch);
		}
		if (drawMode == DRAWMODE_CONTOURS)
		{
			if (g_cset)
				rcDebugDrawContours(*g_cset, g_cfg.bmin, g_cfg.cs, g_cfg.ch);
		}
		
		glDisable(GL_FOG);
		
		if (g_mesh)
		{
			// Agent dimensions.
			const float r = agentRadius;
			const float h = agentHeight;
			float col[4];
			col[0] = 0.6f; col[1] = 0.1f; col[2] = 0.1f; col[3] = 0.75f;
			rcDebugDrawCylinderWire(spos[0]-r, spos[1]+0.02f, spos[2]-r, spos[0]+r, spos[1]+h, spos[2]+r, col);
			
			glColor4ub(0,0,0,196);
			glBegin(GL_LINES);
			glVertex3f(spos[0], spos[1]-agentMaxClimb, spos[2]);
			glVertex3f(spos[0], spos[1]+agentMaxClimb, spos[2]);
			glVertex3f(spos[0]-r/2, spos[1]+0.02f, spos[2]);
			glVertex3f(spos[0]+r/2, spos[1]+0.02f, spos[2]);
			glVertex3f(spos[0], spos[1]+0.02f, spos[2]-r/2);
			glVertex3f(spos[0], spos[1]+0.02f, spos[2]+r/2);
			glEnd();
			
			// Mesh bbox.
			col[0] = 1.0f; col[1] = 1.0f; col[2] = 1.0f; col[3] = 0.25f;
			rcDebugDrawBoxWire(g_cfg.bmin[0], g_cfg.bmin[1], g_cfg.bmin[2],
							   g_cfg.bmax[0], g_cfg.bmax[1], g_cfg.bmax[2], col);
		}
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		
		imguiBeginFrame();
		
		mouseOverMenu = false;
		
		static int propScroll = 0;
		if (imguiBeginScrollArea(GENID, "Properties", width - 250 - 10, 10, 250, height-20, &propScroll))
			mouseOverMenu = true;
		
		if (imguiButton(GENID, curLevel))
		{
			showLevels = true;
			scanDirectory("meshes", ".obj", fileList);
		}
		
		imguiSeparator();
		
		if (g_mesh)
		{
			if (imguiButton(GENID, "Build"))
			{
				memset(&g_cfg, 0, sizeof(g_cfg));
				g_cfg.cs = cellSize;
				g_cfg.ch = cellHeight;
				g_cfg.walkableSlopeAngle = agentMaxSlope;
				g_cfg.walkableHeight = (int)ceilf(agentHeight / g_cfg.ch);
				g_cfg.walkableClimb = (int)ceilf(agentMaxClimb / g_cfg.ch);
				g_cfg.walkableRadius = (int)ceilf(agentRadius / g_cfg.cs);
				g_cfg.maxEdgeLen = (int)(edgeMaxLen / cellSize);
				g_cfg.maxSimplificationError = edgeMaxError;
				g_cfg.minRegionSize = (int)rcSqr(regionMinSize);
				g_cfg.mergeRegionSize = (int)rcSqr(regionMergeSize);
				g_cfg.maxVertsPerPoly = (int)vertsPerPoly;
				
				buildNavigation();
			}
		}
		
		imguiSeparator();
		
		if (imguiCheck(GENID, "Show Log", showLog))
			showLog = !showLog;
		
		imguiSeparator();
		imguiLabel(GENID, "Rasterization");
		imguiSlider(GENID, "Cell Size", &cellSize, 0.1f, 1.0f, 0.01f);
		imguiSlider(GENID, "Cell Height", &cellHeight, 0.1f, 1.0f, 0.01f);
		
		if (g_mesh)
		{
			int gw = 0, gh = 0;
			rcCalcGridSize(g_cfg.bmin, g_cfg.bmax, cellSize, &gw, &gh);
			char text[64];
			snprintf(text, 64, "Grid %d x %d", gw, gh);
			imguiValue(GENID, text);
		}
		
		imguiSeparator();
		imguiLabel(GENID, "Agent");
		imguiSlider(GENID, "Height", &agentHeight, 0.1f, 5.0f, 0.1f);
		imguiSlider(GENID, "Ragius", &agentRadius, 0.1f, 5.0f, 0.1f);
		imguiSlider(GENID, "Max Climb", &agentMaxClimb, 0.1f, 5.0f, 0.1f);
		imguiSlider(GENID, "Max Slope", &agentMaxSlope, 0.0f, 90.0f, 1.0f);
		
		imguiSeparator();
		imguiLabel(GENID, "Region");
		imguiSlider(GENID, "Min Region Size", &regionMinSize, 0.0f, 150.0f, 1.0f);
		imguiSlider(GENID, "Merged Region Size", &regionMergeSize, 0.0f, 150.0f, 1.0f);
		
		imguiSeparator();
		imguiLabel(GENID, "Polygonization");
		imguiSlider(GENID, "Max Edge Length", &edgeMaxLen, 0.0f, 50.0f, 1.0f);
		imguiSlider(GENID, "Max Edge Error", &edgeMaxError, 0.1f, 3.0f, 0.1f);
		imguiSlider(GENID, "Verts Per Poly", &vertsPerPoly, 3.0f, 12.0f, 1.0f);		
		
		imguiSeparator();
		imguiLabel(GENID, "Draw");
		if (imguiCheck(GENID, "Input Mesh", drawMode == DRAWMODE_MESH))
			drawMode = DRAWMODE_MESH;
		if (imguiCheck(GENID, "Navmesh", drawMode == DRAWMODE_POLYMESH))
			drawMode = DRAWMODE_POLYMESH;
		if (imguiCheck(GENID, "Navmesh Trans", drawMode == DRAWMODE_POLYMESH_TRANS))
			drawMode = DRAWMODE_POLYMESH_TRANS;
		if (imguiCheck(GENID, "Voxels", drawMode == DRAWMODE_VOXELS))
			drawMode = DRAWMODE_VOXELS;
		if (imguiCheck(GENID, "Walkable Voxels", drawMode == DRAWMODE_VOXELS_WALKABLE))
			drawMode = DRAWMODE_VOXELS_WALKABLE;
		if (imguiCheck(GENID, "Compact", drawMode == DRAWMODE_COMPACT))
			drawMode = DRAWMODE_COMPACT;
		if (imguiCheck(GENID, "Compact Distance", drawMode == DRAWMODE_COMPACT_DISTANCE))
			drawMode = DRAWMODE_COMPACT_DISTANCE;
		if (imguiCheck(GENID, "Compact Regions", drawMode == DRAWMODE_COMPACT_REGIONS))
			drawMode = DRAWMODE_COMPACT_REGIONS;
		if (imguiCheck(GENID, "Raw Contours", drawMode == DRAWMODE_RAW_CONTOURS))
			drawMode = DRAWMODE_RAW_CONTOURS;
		if (imguiCheck(GENID, "Contours", drawMode == DRAWMODE_CONTOURS))
			drawMode = DRAWMODE_CONTOURS;
		
		imguiEndScrollArea();
		
		// Log
		if (showLog)
		{
			static int logScroll = 0;
			if (imguiBeginScrollArea(GENID, "Log", 10, 10, width - 300, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < g_log.getMessageCount(); ++i)
				imguiLabel(GENID1(i), g_log.getMessageText(i));
			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int scroll = 0;
			if (imguiBeginScrollArea(GENID, "Choose Level", width-10-250-10-200, height-10-250, 200, 250, &scroll))
				mouseOverMenu = true;
			
			int levelToLoad = -1;
			for (int i = 0; i < fileList.size; ++i)
			{
				if (imguiItem(GENID1(i), fileList.files[i]))
					levelToLoad = i;
			}
			
			if (levelToLoad != -1)
			{
				strncpy(curLevel, fileList.files[levelToLoad], sizeof(curLevel));
				curLevel[sizeof(curLevel)-1] = '\0';
				showLevels = false;
				
				delete g_mesh;
				delete g_solid;
				delete g_chf;
				delete g_cset;
				delete g_polyMesh;
				delete [] g_triangleFlags;
				g_mesh = 0;
				g_solid = 0;
				g_chf = 0;
				g_cset = 0;
				g_polyMesh = 0;
				g_triangleFlags = 0;

				g_mesh = new rcMeshLoaderObj;
				
				char path[256];
				strcpy(path, "meshes/");
				strcat(path, curLevel);
				
				if (!g_mesh->load(path))
				{
					printf("Could not load mesh\n");
					delete g_mesh;
					g_mesh = 0;
				}
				
				if (g_mesh)
				{
					rcCalcBounds(g_mesh->getVerts(), g_mesh->getVertCount(), g_cfg.bmin, g_cfg.bmax);
					
					// Reset camera.
					camr = sqrtf(rcSqr(g_cfg.bmax[0]-g_cfg.bmin[0]) +
								 rcSqr(g_cfg.bmax[1]-g_cfg.bmin[1]) +
								 rcSqr(g_cfg.bmax[2]-g_cfg.bmin[2])) / 2;
					camx = (g_cfg.bmax[0] + g_cfg.bmin[0]) / 2 + camr;
					camy = (g_cfg.bmax[1] + g_cfg.bmin[1]) / 2 + camr;
					camz = (g_cfg.bmax[2] + g_cfg.bmin[2]) / 2 + camr;
					camr *= 3;
					rx = 45;
					ry = -45;
					
					glFogf(GL_FOG_START, camr*0.5f);
					glFogf(GL_FOG_END, camr*2.5f);
				}
				
			}
			
			imguiEndScrollArea();
			
		}
		
		imguiEndFrame();
		imguiRender(&drawText);
		
		g_font.drawText(10.0f, (float)height-20.0f, "W/S/A/D: Move  LMB: Rotate  RMB: Place character", GLFont::RGBA(255,255,255,128));
		
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapBuffers();
	}
	
	SDL_Quit();
	
	delete g_mesh;
	delete g_solid;
	delete g_chf;
	delete g_cset;
	delete g_polyMesh;
	delete [] g_triangleFlags;
	
	return 0;
}
