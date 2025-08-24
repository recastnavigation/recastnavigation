#include "AppData.h"

#include "Sample.h"
#include "InputGeom.h"

#include <imgui.h>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

AppData::~AppData()
{
	delete sample;
	sample = nullptr;
	delete inputGeometry;
	inputGeometry = nullptr;
}

void AppData::resetCamera()
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

void AppData::UpdateWindowSize()
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

void AppData::UpdateUIScale()
{
	ImGuiIO& io = ImGui::GetIO();
	io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));
	io.DisplayFramebufferScale = ImVec2(
		static_cast<float>(drawableWidth) / static_cast<float>(width),
		static_cast<float>(drawableHeight) / static_cast<float>(height));
}

void AppData::WorldToScreen(float x, float y, float z, float* screenX, float* screenY)
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
