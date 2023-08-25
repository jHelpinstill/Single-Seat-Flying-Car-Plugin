#include "Matrix.h"


Matrix::Matrix(int n, int m)
{
	makeEmpty(n, m);
	makeIdentity();
}

Matrix::Matrix(float* data[], int n, int m)
{
	create(data, n, m);
}

Matrix::Matrix(const Matrix& A)
{
	this->makeEmpty(A.n, A.m);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			this->a[i][j] = A.a[i][j];
		}
	}
}

void Matrix::makeEmpty(int n, int m)
{
	clear();
	this->n = n;
	this->m = m;
	a = new float* [n];

	for (int row = 0; row < n; row++)
	{
		a[row] = new float[m];
		for (int col = 0; col < m; col++)
		{
			a[row][col] = 0;
		}
	}
}

void Matrix::create(float* data[], int n, int m)
{
	this->clear();
	this->n = n;
	this->m = m;
	a = new float* [n];
	for (int row = 0; row < n; row++)
	{
		a[row] = new float[m];
		for (int col = 0; col < m; col++)
		{
			a[row][col] = data[row][col];
		}
	}
}

void Matrix::clear()
{
	for (int i = 0; i < n; i++)
	{
		delete[] a[i];
	}
	n = m = 0;
}

void Matrix::makeIdentity()
{
	if (n != m)
		return;

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			if (i == j)
				a[i][j] = 1;
			else
				a[i][j] = 0;
		}
	}
}

void Matrix::invert()
{
	if (n != m)
		return;

	Matrix I(n, m);
	I.makeIdentity();
	Matrix A_copy = *this;

	///// make triangular /////
	try
	{
		for (int row = 0; row < n; row++)
		{
			if (a[row][row] == 0)
			{
				for (int i = row + 1; i < n; i++)
				{
					if (a[i][row] != 0)
					{
						float* temp;
						temp = a[row];
						a[row] = a[i];
						a[i] = temp;

						temp = I.a[row];
						I.a[row] = I.a[i];
						I.a[i] = temp;

						break;
					}
					if (i == n - 1)
						throw(MatNonInvertable());
				}
			}
			for (int i = row; i < n; i++)
			{
				float first_element = a[i][row];
				if (first_element == 0)
				{
					continue;
				}
				for (int j = 0; j < m; j++)
				{
					a[i][j] /= first_element;
					I.a[i][j] /= first_element;

				}
				if (i > row)
				{
					for (int j = 0; j < m; j++)
					{
						a[i][j] -= a[row][j];
						I.a[i][j] -= I.a[row][j];
					}
				}
			}
		}

		if (a[n - 1][m - 1] != 1)
		{
			*this = A_copy;
			throw(MatSingular());
		}
	}
	catch (MatSingular e)
	{
		std::cout << "cannot invert: sigular matrix" << std::endl;
		return;
	}
	catch (MatNonInvertable e)
	{
		std::cout << "cannot invert: non-invertable matrix" << std::endl;
		return;
	}

	for (int i = 0; i < n; i++)
	{
		for (int j = i + 1; j < m; j++)
		{
			if (a[i][j] == 0)
				continue;
			float factor = a[i][j];
			for (int col = 0; col < m; col++)
			{
				a[i][col] -= factor * a[j][col];
				I.a[i][col] -= factor * I.a[j][col];
			}
		}
	}

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			a[i][j] = I.a[i][j];
		}
	}
}

float& Matrix::element(int i, int j)
{
	if (i >= n || j >= m)
		throw(MatInvalidOp());
	return a[i][j];
}

Matrix Matrix::operator*(Matrix& A)
{
	if (m != A.n)
		throw(MatInvalidOp());

	Matrix B(n, A.m);
	for (int i = 0; i < B.n; i++)
	{
		for (int j = 0; j < B.m; j++)
		{
			B.a[i][j] = 0;
			for (int k = 0; k < m; k++)
			{
				B.a[i][j] += a[i][k] * A.a[k][j];
			}
		}
	}
	return B;
}

Vec3 Matrix::operator*(Vec3& v)
{
	Matrix V(3, 1);
	for (int i = 0; i < 3; i++)
	{
		V.a[i][0] = v.n[i];
	}
	try
	{
		V = *this * V;
	}
	catch (MatError e)
	{
		exit(0);
	}

	Vec3 u;

	for (int i = 0; i < 3; i++)
	{
		u.n[i] = V.a[i][0];
	}
	return u;
}

void Matrix::operator=(const Matrix& A)
{

	this->makeEmpty(A.n, A.m);

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			a[i][j] = A.a[i][j];
		}
	}
}

void Matrix::print()
{
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			std::cout << a[i][j] << ' ';
		}
		std::cout << std::endl;
	}
}

void Matrix::println()
{
	this->print();
	std::cout << std::endl;
}

Matrix::~Matrix()
{
	this->clear();
}