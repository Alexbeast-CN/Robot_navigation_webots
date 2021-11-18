#include "../lib/Matrix.hpp"

int main(int argc, char const *argv[])
{
    
	Matrix A = Matrix(3, 3);
    std::cin >> A;
    A.Show();
    return 0;
}
