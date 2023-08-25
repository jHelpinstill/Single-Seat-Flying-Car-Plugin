#include "Matrix.h"


Matrix::Matrix(int n, int m)
{
	//std::cout << "empty constructor()" << std::endl;
	makeEmpty(n, m);
	makeIdentity();
	//std::cout << "end of empty constructor()" << std::endl;
}

Matrix::Matrix(float* data[], int n, int m)
{
	//std::cout << "float* data[] constructor()" << std::endl;
	create(data, n, m);
	//std::cout << "end of float* data[] constructor()" << std::endl;
}

Matrix::Matrix(const Matrix& A)
{
	//std::cout << "copy constructor()" << std::endl;
	this->makeEmpty(A.n, A.m);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			this->a[i][j] = A.a[i][j];
		}
	}
	//std::cout << "end of copy constructor()" << std::endl;
}

void Matrix::makeEmpty(int n, int m)
{
	//std::cout << "makeEmpty()" << std::endl;
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
	//std::cout << "end of makeEmpty()" << std::endl;
}

void Matrix::create(float* data[], int n, int m)
{
	//std::cout << "create()" << std::endl;
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
	//std::cout << "end of create()" << std::endl;
}

void Matrix::clear()
{
	//std::cout << "clear()" << std::endl;
	for (int i = 0; i < n; i++)
	{
		delete[] a[i];
	}
	n = m = 0;
	//std::cout << "end of clear()" << std::endl;
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
	//std::cout << "invert()" << std::endl;
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
				//				std::cout << "swapped rows " << row;
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

						//						std::cout << " and " << i << std::endl;
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
					//					std::cout << "\n skipped row " << row << std::endl;
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
				//				std::cout << "A" << std::endl;
				//				this->println();
				//				std::cout << "I" << std::endl;
				//				I.println();
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
			//			std::cout << "A" << std::endl;
			//			this->println();
			//			std::cout << "I" << std::endl;
			//			I.println();
		}
	}

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			a[i][j] = I.a[i][j];
		}
	}

	//std::cout << "end of invert()" << std::endl;
}

float& Matrix::element(int i, int j)
{
	if (i >= n || j >= m)
		throw(MatInvalidOp());
	return a[i][j];
}

Matrix Matrix::operator*(Matrix& A)
{
	//	std::cout << "before throw" << std::endl;
	if (m != A.n)
		throw(MatInvalidOp());
	//	std::cout << "after throw" << std::endl;
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
		//		std::cout << "Error: " << e.warning();
		exit(0);
	}

	Vec3 u;

	for (int i = 0; i < 3; i++)
	{
		u.n[i] = V.a[i][0];
	}
	return u;
}

void Matrix::operator=(Matrix A)
{
	//std::cout << "operator=()" << std::endl;
//	this->clear();	
	this->makeEmpty(A.n, A.m);

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			a[i][j] = A.a[i][j];
		}
	}
	//std::cout << "end of operator=()" << std::endl;
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
	//std::cout << "destructor()" << std::endl;
	this->clear();
	//std::cout << "end of destructor()" << std::endl;
}