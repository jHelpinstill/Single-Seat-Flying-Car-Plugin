#pragma once
#ifndef CONTROL_MATRIX
#define COTNROL_MATRIX

#include "Matrix.h"
#include "Vec3.h"

class ControlMatrix
{
public:
	Matrix mat = Matrix(6, 6);
	
public:
	
	ControlMatrix() {}
	void fillMatrix(Vec3 nose_fan_pos, Vec3 left_pos, Vec3 right_pos, Vec3 CoM_pos);
	void compute(Vec3 thrust, Vec3 moment, Vec3& left_fan, Vec3& right_fan, Vec3& nose_fan);

};

#endif