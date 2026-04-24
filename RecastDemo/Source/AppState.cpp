#include "AppState.h"
#include "InputGeom.h"
#include "SDL_opengl.h"
#include "Sample.h"

#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <imgui.h>

void AppState::resetCamera()
{
	// Reset camera and fog to match the mesh bounds.
	if (inputGeometry)
	{
		const float* boundsMin = inputGeometry->getNavMeshBoundsMin();
		const float* boundsMax = inputGeometry->getNavMeshBoundsMax();

		camr = rcSqr(boundsMax[0] - boundsMin[0]);
		camr += rcSqr(boundsMax[1] - boundsMin[1]);
		camr += rcSqr(boundsMax[2] - boundsMin[2]);
		camr = sqrtf(camr) / 2.0f;

		cameraPos[0] = (boundsMax[0] + boundsMin[0]) / 2.0f + camr;
		cameraPos[1] = (boundsMax[1] + boundsMin[1]) / 2.0f + camr;
		cameraPos[2] = (boundsMax[2] + boundsMin[2]) / 2.0f + camr;

		camr *= 3.0f;
	}
	cameraEulers[0] = 45;
	cameraEulers[1] = -45;
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
}

void AppState::saveCamera()
{
	savedCameraSet = true;
	savedCamr = camr;
	savedCameraEulers[0] = cameraEulers[0];
	savedCameraEulers[1] = cameraEulers[1];
	savedCameraPos[0] = cameraPos[0];
	savedCameraPos[1] = cameraPos[1];
	savedCameraPos[2] = cameraPos[2];
}

bool AppState::restoreSavedCamera()
{
	if (!savedCameraSet)
	{
		return false;
	}

	camr = savedCamr;
	cameraEulers[0] = savedCameraEulers[0];
	cameraEulers[1] = savedCameraEulers[1];
	cameraPos[0] = savedCameraPos[0];
	cameraPos[1] = savedCameraPos[1];
	cameraPos[2] = savedCameraPos[2];

	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	return true;
}

void AppState::setSelectedPoint(const float* point)
{
	if (!point)
	{
		return;
	}

	selectedPointSet = true;
	selectedPoint[0] = point[0];
	selectedPoint[1] = point[1];
	selectedPoint[2] = point[2];
}

void AppState::requestFocusSelectedPoint()
{
	if (selectedPointSet)
	{
		focusSelectedPointRequested = true;
	}
}

void AppState::focusCameraOnSelectedPoint(const double* modelviewMatrix)
{
	if (!selectedPointSet || !modelviewMatrix)
	{
		return;
	}

	float focusDistance = camr * 0.1f;
	if (focusDistance < 6.0f)
	{
		focusDistance = 6.0f;
	}
	else if (focusDistance > 35.0f)
	{
		focusDistance = 35.0f;
	}

	cameraPos[0] = selectedPoint[0] + static_cast<float>(modelviewMatrix[2]) * focusDistance;
	cameraPos[1] = selectedPoint[1] + static_cast<float>(modelviewMatrix[6]) * focusDistance;
	cameraPos[2] = selectedPoint[2] + static_cast<float>(modelviewMatrix[10]) * focusDistance;
	focusSelectedPointRequested = false;
}

void AppState::updateWindowSize()
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

void AppState::updateUIScale() const
{
	ImGuiIO& io = ImGui::GetIO();
	io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));
	io.DisplayFramebufferScale = ImVec2(
		static_cast<float>(drawableWidth) / static_cast<float>(width),
		static_cast<float>(drawableHeight) / static_cast<float>(height));
}

void AppState::worldToScreen(float x, float y, float z, float* screenX, float* screenY) const
{
	GLdouble modelviewMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);

	GLdouble winX, winY, winZ;
	gluProject(x, y, z, modelviewMatrix, projectionMatrix, viewport, &winX, &winY, &winZ);

	const float dpiScaleX = static_cast<float>(drawableWidth) / static_cast<float>(width);
	const float dpiScaleY = static_cast<float>(drawableHeight) / static_cast<float>(height);

	*screenX = static_cast<float>(winX) / dpiScaleX;
	*screenY = static_cast<float>(height) - static_cast<float>(winY / dpiScaleY);
}
