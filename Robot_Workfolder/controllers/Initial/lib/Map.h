#ifndef _MAP_H
#define _MAP_H

#include "Matrix.h"

class Map
{
private:
    /* data */
public:
    Map(/* args */);
    ~Map();
    Matrix easyMap();
    Matrix easyMapS();
};

Map::Map(/* args */)
{
}

Map::~Map()
{
}

Matrix Map::easyMap()
{
    Matrix easyMap(92,92);
    for (int i=0; i<92; i++)
    {
        easyMap.p[i][0] = 1;
        easyMap.p[i][91] = 1;
        easyMap.p[0][i] = 1;
        easyMap.p[91][i] = 1;
    }

    for (int i=1; i<=30; i++)
        for (int j=1; j<=30; j++)
        {
            easyMap.p[30+i][20+j] = 1;
        }

    for (int i=1; i<=10; i++)
        for (int j=1; j<=20; j++)
        {
            easyMap.p[40+i][20+j] = 0;
        }
    return easyMap;
}

Matrix Map::easyMapS()
{
    Matrix easyMap(11,11);
    for (int i=0; i<11; i++)
    {
        easyMap.p[i][0] = 1;
        easyMap.p[i][10] = 1;
        easyMap.p[0][i] = 1;
        easyMap.p[10][i] = 1;
    }

    for (int i=1; i<=3; i++)
        for (int j=0; j<=3; j++)
        {
            easyMap.p[3+i][2+j] = 1;
        }

    for (int j=1; j<=2; j++)
    {
        easyMap.p[5][2+j] = 0;
    }
    return easyMap;
}


#endif