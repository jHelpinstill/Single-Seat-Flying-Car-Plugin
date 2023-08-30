#pragma once

#include "XPLMGraphics.h"
#include "Vec3.h"
#include "Matrix.h"
#include "Quat.h"

class TextBox
{
private:
	int x = 0;
	int y = 0;

	int line_height = 10;

	int cursor_x = 0;
	int cursor_y = 0;

	float col_white[3] = { 1.0, 1.0, 1.0 };
	int float_precision = 3;

	XPLMFontID font = xplmFont_Proportional;

public:
	TextBox() {};
	TextBox(int x, int y);

	void setPrecision(int precision);

	template <class T>
	inline void println(char* s, T o)
	{
		print(s);
		println(o);
	}
	template <class T>
	inline void print(char* s, T o)
	{
		print(s);
		print(o);
	}

	void print(Vec3 v);
	void println(Vec3 v);
	void print(int num);
	void println(int num);
	void print(float data);
	void println(float d);
	void print(char* s);
	void print(Matrix& A);
	void print(Quat q);
	void println(char* s);
	void println();
	void reset(int x = 0, int y = 0);
};
