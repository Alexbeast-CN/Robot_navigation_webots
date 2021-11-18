#include  <iostream>

using namespace std;

int main(int argc, char const *argv[])
{
    int x=9,y=9;
    int a[x][y];

    for(int i=0; i<x; i++)
    {
        for(int j=0; j<y; j++)
        {
            a[i][j] = 0;
            cout << a[i][j] << " ";
        }
        cout << "\n";
    }
    return 0;
}

