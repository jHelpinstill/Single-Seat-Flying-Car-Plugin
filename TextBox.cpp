#include "TextBox.h"

TextBox::TextBox(int x, int y)
{
	this->x = x;
	this->y = y;
}

void TextBox::setPrecision(int precision)
{
	if (precision < 1) return;

	float_precision = precision;
}

void TextBox::print(Vec3 v)
{
	print("x: ");
	println(v.x);
	print("y: ");
	println(v.y);
	print("z: ");
	println(v.z);
}
void TextBox::println(Vec3 v)
{
	println();
	print(v);
	println();
}
void TextBox::print(int num)
{
	int num_digits = log10(num) + 1;
	XPLMDrawNumber(col_white, x + cursor_x, y + cursor_y, num, num_digits, 0, true, font);
	int width;
	XPLMGetFontDimensions(font, &width, NULL, NULL);
	width *= num_digits + 1;
	cursor_x += width;
}
void TextBox::println(int num)
{
	print(num);
	println();
}
void TextBox::print(float data)
{
	int num_digits = log10((int)data);
	num_digits += float_precision + 1;
	XPLMDrawNumber(col_white, x + cursor_x, y + cursor_y, data, num_digits, float_precision, true, font);

	int width;
	XPLMGetFontDimensions(font, &width, NULL, NULL);
	width *= num_digits + 1;
	cursor_x += width;
}
void TextBox::println(float d)
{
	print(d);
	println();
}
void TextBox::print(char* s)
{
	XPLMDrawString(col_white, x + cursor_x, y + cursor_y, s, NULL, font);
	int length = 1;
	int i = 0;
	int offset;
	while (s[i] != 0)
	{
		length++;
		i++;
	}
	cursor_x += XPLMMeasureString(font, s, length);
}
void TextBox::print(Matrix& A)
{
	for (int i = 0; i < A.n; i++)
	{
		for (int j = 0; j < A.m; j++)
		{
			print(A.a[i][j]);
		}
		println();
	}
}
void TextBox::print(Quat q)
{
	println();
	print("w: ");
	println(q.w);
	print("x: ");
	println(q.x);
	print("y: ");
	println(q.y);
	print("z: ");
	println(q.z);
}
void TextBox::println(char* s)
{
	print(s);
	println();
}
void TextBox::println()
{
	cursor_x = 0;
	cursor_y -= line_height;
}
void TextBox::reset(int x, int y)
{
	this->x = x;
	this->y = y;
	this->cursor_x = 0;
	this->cursor_y = 0;
}