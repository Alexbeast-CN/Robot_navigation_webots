#ifndef _MATRIX_H
#define _MATRIX_H

#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <cmath>

class Matrix {

private:
	int rows_num, cols_num;
	double **p;
	void initialize();//Initialization matrix
 
public:
	Matrix(int, int);
	Matrix(int, int, double);//Pre-match space
	virtual ~Matrix();//The destructor should be a virtual function, unless you don't have to do a base class.
	Matrix& operator=(const Matrix&);//Matrix copy
	Matrix& operator=(double *);//Pass the value of the array to the matrix
	Matrix& operator+=(const Matrix&);//Matrix + = operation
	Matrix& operator-=(const Matrix&);//-=
	void Show() const;//Matrix display
	void swapRows(int, int);
	double Point(int i, int j) const;
	static Matrix eye(int );//Manufacturing a unit matrix
	int row() const;
	int col() const;
	static Matrix T(const Matrix & m);//Implementation of matrix transposition and does not change matrix

	friend std::istream& operator>>(std::istream&, Matrix&);//Implement matrix input
	friend class Map;
};

using std::endl;
using std::cout;
using std::istream;
const double EPS = 1e-10;

void Matrix::initialize() {//Initialization matrix size
	p = new double*[rows_num];//Assign ROWS_NUM pointer
	for (int i = 0; i < rows_num; ++i) {
		p[i] = new double[cols_num];//Dynamic memory allocation for P [i], size is cols
	}
}

// Declear an all zero matrix
Matrix::Matrix(int rows, int cols)
{
	rows_num = rows;
	cols_num = cols;
	initialize();
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			p[i][j] = 0;
		}
	}
}
// Declear a matrix with all element of a vaule 
Matrix::Matrix(int rows, int cols, double value)
{
	rows_num = rows;
	cols_num = cols;
	initialize();
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			p[i][j] = value;
		}
	}
}
 
Matrix::~Matrix() {
 for (int i = 0; i < rows_num; ++i) {
			delete[] p[i];
		}
		delete[] p;
}


// achive matrix copy
Matrix& Matrix::operator=(const Matrix& m)
{
	if (this == &m) {
		return *this;
	}
 
	if (rows_num != m.rows_num || cols_num != m.cols_num) {
		for (int i = 0; i < rows_num; ++i) {
			delete[] p[i];
		}
		delete[] p;
 
		rows_num = m.rows_num;
		cols_num = m.cols_num;
		initialize();
	}
 
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			p[i][j] = m.p[i][j];
		}
	}
	return *this;
}


// Pass the value of the array to the matrix 
// (requires that the size of the matrix has been declared)
Matrix& Matrix::operator=(double *a){
	for(int i=0;i<rows_num;i++){
		for(int j=0;j<cols_num;j++){
			p[i][j]= *(a+i*cols_num+j);
		}
	}
	return *this;
}

// achieve += for matrix
Matrix& Matrix::operator+=(const Matrix& m)
{
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			p[i][j] += m.p[i][j];
		}
	}
	return *this;
}

// achieve-=
Matrix& Matrix::operator-=(const Matrix& m)
{
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			p[i][j] -= m.p[i][j];
		}
	}
	return *this;
}

// Display the martix
void Matrix::Show() const {
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			cout << p[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

//Implementation transformation
void Matrix::swapRows(int a, int b)
{
	a--;
	b--;
	double *temp = p[a];
	p[a] = p[b];
	p[b] = temp;
}

//Returns the number of number J columns in group I of the matrix
double Matrix::Point(int i, int j) const{
	return this->p[i][j];
}

//Make an unit matrix
Matrix Matrix::eye(int n){
	Matrix A(n,n);
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++){
			if(i==j){
				A.p[i][j]=1;
			}else{
				A.p[i][j]=0;
			}
		}
	}
	return A;
}

//Read matrix ranks
int Matrix::row() const{
	return rows_num;
}
int Matrix::col() const{
	return cols_num;
}

//Implementation matrix transposition
Matrix Matrix::T(const Matrix & m)
{	int col_size=m.col();
	int row_size=m.row();
	Matrix mt(col_size, row_size);
	for (int i = 0; i <row_size; i++) {
		for (int j = 0; j <col_size; j++) {
			mt.p[j][i] = m.p[i][j];
		}
	}
	return mt;
}

//Implement matrix input
istream& operator>>(istream& is, Matrix& m)
{
	for (int i = 0; i < m.rows_num; i++) {
		for (int j = 0; j < m.cols_num; j++) {
			is >> m.p[i][j];
		}
	}
	return is;
}

#endif