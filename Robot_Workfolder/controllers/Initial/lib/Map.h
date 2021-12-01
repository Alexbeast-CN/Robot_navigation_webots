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
    Matrix markTrajectory(int x, int y);
    Matrix markTrajectoryS(int x, int y);


};

Map::Map(/* args */)
{
    cout<<"A map has been created!" << endl;
}

Map::~Map()
{
}

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
Matrix Map::markTrajectoryS(int x, int y)
{
    Matrix map(11,11);
    map.p[x][y] = TRAJECTORY;
    return map;
}

Matrix Map::markTrajectory(int x, int y)
{
    Matrix map(92,92);
    for(int i=-4; i<=4; i++)
        for(int j=-4; j<=4; j++)
            map.p[x+i][y+j] = TRAJECTORY;
    return map;
}   



#endif