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
	void initialize();//初始化矩阵
 
public:
	Matrix(int, int);
	Matrix(int, int, double);//预配分空间
	virtual ~Matrix();//析构函数应当是虚函数，除非此类不用做基类
	Matrix& operator=(const Matrix&);//矩阵的复制
	Matrix& operator=(double *);//将数组的值传给矩阵
	// Matrix& operator+=(const Matrix&);//矩阵的+=操作
	// Matrix& operator-=(const Matrix&);//-=
	// Matrix& operator*=(const Matrix&);//*=
	// Matrix operator*(const Matrix & m)const;
	void Show() const;//矩阵显示
	void swapRows(int, int);
	double Point(int i, int j) const;
	static Matrix inv(Matrix);//求矩阵的逆矩阵
	static Matrix eye(int );//制造一个单位矩阵
	int row() const;
	int col() const;
	static Matrix T(const Matrix & m);//矩阵转置的实现,且不改变矩阵

	friend std::istream& operator>>(std::istream&, Matrix&);//实现矩阵的输入
	friend class Map;
};

using std::endl;
using std::cout;
using std::istream;
const double EPS = 1e-10;

void Matrix::initialize() {//初始化矩阵大小
	p = new double*[rows_num];//分配rows_num个指针
	for (int i = 0; i < rows_num; ++i) {
		p[i] = new double[cols_num];//为p[i]进行动态内存分配，大小为cols
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
/*
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


// achieve*=
Matrix& Matrix::operator*=(const Matrix& m)
{
	Matrix temp(rows_num, m.cols_num);//若C=AB,则矩阵C的行数等于矩阵A的行数，C的列数等于B的列数。
	for (int i = 0; i < temp.rows_num; i++) {
		for (int j = 0; j < temp.cols_num; j++) {
			for (int k = 0; k < cols_num; k++) {
				temp.p[i][j] += (p[i][k] * m.p[k][j]);
			}
		}
	}
	*this = temp;
	return *this;
}


// achieve matrix dot product
Matrix Matrix::operator*(const Matrix & m)const{
	Matrix ba_M(rows_num,m.cols_num,0.0);
	for(int i=0;i<rows_num;i++){
		for(int j=0;j<m.cols_num;j++){
			for(int k=0;k<cols_num;k++){
				ba_M.p[i][j]+=(p[i][k]*m.p[k][j]);
			}
		}
	}
	return ba_M;
}
*/

// Display the martix
void Matrix::Show() const {
	//cout << rows_num <<" "<<cols_num<< endl;//显示矩阵的行数和列数
	for (int i = 0; i < rows_num; i++) {
		for (int j = 0; j < cols_num; j++) {
			cout << p[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

//实现行变换
void Matrix::swapRows(int a, int b)
{
	a--;
	b--;
	double *temp = p[a];
	p[a] = p[b];
	p[b] = temp;
}

//返回矩阵第i行第j列的数
double Matrix::Point(int i, int j) const{
	return this->p[i][j];
}
//求矩阵的逆矩阵
Matrix Matrix::inv(Matrix A){
	if(A.rows_num!=A.cols_num){
		std::cout<<"只有方阵能求逆矩阵"<<std::endl;
		std::abort();//只有方阵能求逆矩阵
	}
	double temp;
	Matrix A_B=Matrix(A.rows_num,A.cols_num);
	A_B=A;//为矩阵A做一个备份
	Matrix B=eye(A.rows_num);
	//将小于EPS的数全部置0
	for (int i = 0; i < A.rows_num; i++) {
		for (int j = 0; j < A.cols_num; j++) {
			if (abs(A.p[i][j]) <= EPS) {
				A.p[i][j] = 0;
			}
		}
	}
	//选择需要互换的两行选主元
	for(int i=0;i<A.rows_num;i++){
		if(abs(A.p[i][i])<=EPS){
			bool flag=false;
			for(int j=0;(j<A.rows_num)&&(!flag);j++){
				if((abs(A.p[i][j])>EPS)&&(abs(A.p[j][i])>EPS)){
					flag=true;
					for(int k=0;k<A.cols_num;k++){
						temp=A.p[i][k];
						A.p[i][k]=A.p[j][k];
						A.p[j][k]=temp;
						temp=B.p[i][k];
						B.p[i][k]=B.p[j][k];
						B.p[j][k]=temp;
					}
				}
			}
			if(!flag){
				std::cout<<"逆矩阵不存在\n";
				std::abort();
			}
		}
	}
	//通过初等行变换将A变为上三角矩阵
	double temp_rate;
	for(int i=0;i<A.rows_num;i++){
		for(int j=i+1;j<A.rows_num;j++){
			temp_rate=A.p[j][i]/A.p[i][i];
			for(int k=0;k<A.cols_num;k++){
				A.p[j][k]-=A.p[i][k]*temp_rate;
				B.p[j][k]-=B.p[i][k]*temp_rate;
			}
			A.p[j][i]=0;
		}
	}
	//使对角元素均为1
	for(int i=0;i<A.rows_num;i++){
		temp=A.p[i][i];
		for(int j=0;j<A.cols_num;j++){
			A.p[i][j]/=temp;
			B.p[i][j]/=temp;
		}
	}
	//std::cout<<"算法可靠性检测，若可靠，输出上三角矩阵"<<std::endl;
	//将已经变为上三角矩阵的A，变为单位矩阵
	for(int i=A.rows_num-1;i>=1;i--){
		for(int j=i-1;j>=0;j--){
			temp=A.p[j][i];
			for(int k=0;k<A.cols_num;k++){
				A.p[j][k]-=A.p[i][k]*temp;
				B.p[j][k]-=B.p[i][k]*temp;
			}
		}
	}
	std::cout<<"算法可靠性检测，若可靠，输出单位矩阵"<<std::endl;
	for(int i=0;i<A.rows_num;i++){
		for(int j=0;j<A.cols_num;j++){
			printf("%7.4lf\t\t",A.p[i][j]);
		}
		cout << endl;
	}
	A=A_B;//还原A
	return B;//返回该矩阵的逆矩阵
}
//制造一个单位矩阵
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

//读取矩阵行列数
int Matrix::row() const{
	return rows_num;
}
int Matrix::col() const{
	return cols_num;
}

//实现矩阵的转置
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

//实现矩阵的输入
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