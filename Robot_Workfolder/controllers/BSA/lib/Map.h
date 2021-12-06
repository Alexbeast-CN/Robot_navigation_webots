#ifndef _MAP_H
#define _MAP_H

#include "Matrix.h"
#include <vector>

#define BLANK  0
#define WALL 1
#define TRAJECTORY 2

using namespace std;

class Map
{
private:


public:
    Map(/* args */);
    ~Map();
    Matrix easyMap();
    Matrix easyMapS();
    Matrix easyMapSS();
    Matrix hardMap();
    Matrix midMapS();
    Matrix markTrajectory(int x, int y);
    Matrix markTrajectoryS(int x, int y);
    Matrix markTrajectoryB(int x, int y);
    Matrix markTrajectoryH(int x, int y);
};

Map::Map(/* args */)
{
    cout<<"A map has been created!" << endl;
}

Map::~Map()
{
}

// 92x92 version of easyMap
Matrix Map::easyMap()
{
    Matrix easyMap(92,92);
    for (int i=0; i<92; i++)
    {
        easyMap.p[i][0] = WALL;
        easyMap.p[i][91] = WALL;
        easyMap.p[0][i] = WALL;
        easyMap.p[91][i] = WALL;
    }

    for (int i=1; i<=30; i++)
        for (int j=1; j<=30; j++)
        {
            easyMap.p[30+i][20+j] = WALL;
        }

    for (int i=1; i<=10; i++)
        for (int j=1; j<=20; j++)
        {
            easyMap.p[40+i][20+j] = BLANK;
        }
    return easyMap;
}

// 11x11 version of easyMap
Matrix Map::easyMapS()
{
    Matrix easyMap(11,11);
    for (int i=0; i<11; i++)
    {
        easyMap.p[i][0] = WALL;
        easyMap.p[i][10] = WALL;
        easyMap.p[0][i] = WALL;
        easyMap.p[10][i] = WALL;
    }

    for (int i=1; i<=3; i++)
        for (int j=1; j<=3; j++)
        {
            easyMap.p[3+i][2+j] = WALL;
        }

    for (int j=1; j<=2; j++)
    {
        easyMap.p[5][2+j] = BLANK;
    }
    return easyMap;
}

Matrix Map::midMapS()
{
    Matrix midMap(11,11);
    for (int i=0; i<11; i++)
    {
        midMap.p[i][0] = WALL;
        midMap.p[i][10] = WALL;
        midMap.p[0][i] = WALL;
        midMap.p[10][i] = WALL;
    }

    midMap.p[2][2] = WALL;
    midMap.p[2][7] = WALL;
    midMap.p[3][2] = WALL;
    midMap.p[8][3] = WALL;
    midMap.p[8][4] = WALL;
    midMap.p[8][7] = WALL;
    return midMap;
}

    Matrix Map::easyMapSS()
{
    Matrix easyMap(11,11);
    for (int i=0; i<11; i++)
    {
        easyMap.p[i][0] = WALL;
        easyMap.p[i][10] = WALL;
        easyMap.p[0][i] = WALL;
        easyMap.p[10][i] = WALL;
    }

    for (int i=1; i<=3; i++)
        for (int j=1; j<=3; j++)
        {
            easyMap.p[3+i][2+j] = WALL;
        }

    for (int j=1; j<=2; j++)
    {
        easyMap.p[5][2+j] = BLANK;
    }
        return easyMap;
    }

Matrix Map::hardMap()
{
    Matrix hardMap(29,29);
    for (int i=0; i<29; i++)
    {
        hardMap.p[i][0] = WALL;
        hardMap.p[i][29] = WALL;
        hardMap.p[0][i] = WALL;
        hardMap.p[29][i] = WALL;
    }

    for (int i=0; i<9; i++)
    {
        hardMap.p[9][1+i] = WALL;
        hardMap.p[1+i][11] = WALL;
        hardMap.p[9][11+i] = WALL;
        hardMap.p[12+i][4] = WALL;
        hardMap.p[16][5+i] = WALL;
        hardMap.p[12+i][16] = WALL;
        hardMap.p[16][18+i] = WALL;
    }
    for (int i = 0; i < 5; i++)
    {
        hardMap.p[5][16+i] = WALL;
        hardMap.p[20+i][23] = WALL;
    }
    for (int i = 0; i < 11; i++)
    {
        hardMap.p[3+i][24] = WALL;
        hardMap.p[23][8+i] = WALL;
        
    }
    
    return hardMap;
}


// markTrajectory for 11x11 easyMap
Matrix Map::markTrajectoryS(int x, int y)
{
    Matrix map(11,11);
    map.p[x][y] = TRAJECTORY;
    return map;
}

Matrix Map::markTrajectoryB(int x, int y)
{
    Matrix map(11,11);
    map.p[x][y] = -2;
    return map;
}

// markTrajectory for 92x92 easyMap
Matrix Map::markTrajectory(int x, int y)
{
    Matrix map(92,92);
    for(int i=-4; i<=4; i++)
        for(int j=-4; j<=4; j++)
            map.p[x+i][y+j] = TRAJECTORY;
    return map;
}   

// markTrajectory for 29x29 hardMap
Matrix Map::markTrajectoryH(int x, int y)
{
    Matrix map(29,29);
    map.p[x][y] = TRAJECTORY;
    return map;
} 

#endif