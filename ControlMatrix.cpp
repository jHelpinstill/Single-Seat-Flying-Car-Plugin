#include "ControlMatrix.h"

void ControlMatrix::fillMatrix(Vec3 nose_fan_pos, Vec3 left_pos, Vec3 right_pos, Vec3 CoM_pos)
{
	nose_fan_pos -= CoM_pos;
	left_pos -= CoM_pos;
	right_pos -= CoM_pos;
	float* data[6] =
	{
		new float[6]{0, 1, 0, 1, 0, 0},
		new float[6]{0, 0, 0, 0, 0, 2},
		new float[6]{1, 0, 1, 0, 1, 0},
		new float[6]{nose_fan_pos.y, 0, left_pos.y, 0, right_pos.y, -(left_pos.z + right_pos.z)},
		new float[6]{-nose_fan_pos.x, left_pos.z, -left_pos.x, right_pos.z, -right_pos.x, 0},
		new float[6]{0, -left_pos.y, 0, -right_pos.y, 0, (left_pos.x + right_pos.x)},
	};

	mat = Matrix(data, 6, 6);
	mat.invert();
}

void ControlMatrix::getData(Vec3 thrust, Vec3 moment, Vec3& left_fan, Vec3& right_fan, Vec3& nose_fan)
{
	Matrix v(6, 1);
	for (int i = 0; i < 3; i++)
	{
		v.a[i][0] = thrust.n[i];
		v.a[i + 3][0] = moment.n[i];
	}

	v = mat * v;

	left_fan = Vec3(v.a[1][0], v.a[5][0], v.a[2][0]);
	right_fan = Vec3(v.a[3][0], v.a[5][0], v.a[4][0]);
	nose_fan = Vec3(0, 0, v.a[0][0]);
}