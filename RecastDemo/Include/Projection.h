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

#ifndef PROJECTION_H
#define PROJECTION_H

inline void mmul(float* m, const double* a, const double* b)
{
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++)
			m[r*4+c] =
					a[r*4+0] * b[0*4+c] +
					a[r*4+1] * b[1*4+c] +
					a[r*4+2] * b[2*4+c] +
					a[r*4+3] * b[3*4+c];
}

inline void midentity(float* m)
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			m[i*4+j] = i == j ? 1 : 0;
}

static void minverse(float* inv, const float* src)
{
	int i, j, k;
	float t;
	float temp[4][4];

	for (i = 0; i < 4; i++)
		for (j = 0; j  <4; j++)
			temp[i][j] = src[i*4+j];
	midentity(inv);

	for (i = 0; i < 4; i++)
	{
		if (temp[i][i] == 0.0f)
		{
			for (j = i + 1; j < 4; j++)
				if (temp[j][i] != 0.0f)
					break;

			if (j != 4)
			{
				for (k = 0; k < 4; k++)
				{
					t = temp[i][k];
					temp[i][k] = temp[j][k];
					temp[j][k] = t;
					t = inv[i*4+k];
					inv[i*4+k] = inv[j*4+k];
					inv[j*4+k] = t;
				}
			}
			else
			{
				return;
			}
		}

		t = 1.0f / temp[i][i];
		for (k = 0; k < 4; k++)
		{
			temp[i][k] *= t;
			inv[i*4+k] *= t;
		}

		for (j = 0; j < 4; j++)
		{
			if (j != i)
			{
				t = temp[j][i];
				for (k = 0; k < 4; k++)
				{
					temp[j][k] -= temp[i][k]*t;
					inv[j*4+k] -= inv[i*4+k]*t;
				}
			}
		}
	}
}

inline void transpoint4(float* r, const float* m, const float* v)
{
	r[0] = v[0]*m[0] + v[1]*m[4] + v[2]*m[8] + v[3]*m[12];
	r[1] = v[0]*m[1] + v[1]*m[5] + v[2]*m[9] + v[3]*m[13];
	r[2] = v[0]*m[2] + v[1]*m[6] + v[2]*m[10] + v[3]*m[14];
	r[3] = v[0]*m[3] + v[1]*m[7] + v[2]*m[11] + v[3]*m[15];
}

inline bool project(float x, float y, float z, const double* model,
				const double* proj, const int* view, float* pos)
{
	float A[16];
	float in[4], out[4];

	in[0] = x;
	in[1] = y;
	in[2] = z;
	in[3] = 1;

	mmul(A, model, proj);
	transpoint4(out, A, in);
	if (out[3] == 0.0f)
		return false;

	pos[0] = ((out[0] / out[3]) + 1) * view[0] / 2;
	pos[1] = ((out[1] / out[3]) + 1) * view[1] / 2;
	pos[2] =  (out[2] / out[3]);
	
	return true;
}

inline bool unproject(float wx, float wy, float wz,
				const double* model, const double* proj,
				const int* view, float* pos)
{
	float m[16], A[16];
	float in[4], out[4];

	in[0] = (wx - (float)view[0]) / (float)view[2] * 2.0 - 1.0;
	in[1] = (wy - (float)view[1]) / (float)view[3] * 2.0 - 1.0;
	in[2] = 2.0 * wz-1.0;
	in[3] = 1.0;

	mmul(A, model, proj);
	minverse(m, A);
	transpoint4(out, m, in);

	if (out[3] == 0.0f)
		return false;

	out[3] = 1.0 / out[3];
	pos[0] = out[0] * out[3];
	pos[1] = out[1] * out[3];
	pos[2] = out[2] * out[3];
	return true;
}

inline void frustum(float *m, float left, float right,
		 		float bottom, float top, float znear, float zfar)
{
	const float nx2 = 2.0 * znear;
	const float drl = right - left;
	const float dtb = top - bottom;
	const float dfn = zfar - znear;

	*m++ = nx2 / drl;
	*m++ = 0.0f;
	*m++ = 0.0f;
	*m++ = 0.0f;

	*m++ = 0.0f;
	*m++ = nx2 / dtb;
	*m++ = 0.0f;
	*m++ = 0.0f;

	*m++ = (right + left) / drl;
	*m++ = (top + bottom) / dtb;
	*m++ = (-zfar - znear) / dfn;
	*m++ = -1.0f;

	*m++ = 0.0f;
	*m++ = 0.0f;
	*m++ = (-nx2 * zfar) / dfn;
	*m++ = 0.0f;
}

inline void perspective(float* m, const float &fovy, const float &aspect,
				const float &znear, const float &zfar)
{
	const float ymax = znear * tanf(fovy * M_PI / 360.0);
	const float xmax = ymax * aspect;
	frustum(m, -xmax, xmax, -ymax, ymax, znear, zfar);
}

inline void ortho2d(float* m, float left, float right,
				float bottom, float top)
{
	const float znear = -1.0f;
	const float zfar = 1.0f;
	const float inv_z = 1.0f / (zfar - znear);
	const float inv_y = 1.0f / (top - bottom);
	const float inv_x = 1.0f / (right - left);

	*m++ = 2.0f * inv_x;
	*m++ = 0.0f;
	*m++ = 0.0f;
	*m++ = 0.0f;

	*m++ = 0.0f;
	*m++ = 2.0 * inv_y;
	*m++ = 0.0f;
	*m++ = 0.0f;

	*m++ = 0.0f;
	*m++ = 0.0f;
	*m++ = -2.0f * inv_z;
	*m++ = 0.0f;

	*m++ = -(right + left) * inv_x;
	*m++ = -(top + bottom) * inv_y;
	*m++ = -(zfar + znear) * inv_z;
	*m++ = 1.0f;
}

#endif // PROJECTION_H
