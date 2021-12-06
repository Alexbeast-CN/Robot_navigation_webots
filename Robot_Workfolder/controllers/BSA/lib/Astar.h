#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <cmath>
#include <queue>
#include <utility>
#include <cstdio>
#include <map>

#include "Matrix.h"
#include "Map.h"

// #define mp(x,y) make_pair(x,y)

class Astar
{
private:
    int mp[11][11], h[11][11], g[11][11], v[11][11];
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, 1, 0, -1};
    std::pair<int, int> s, e; 
    std::priority_queue<std::pair<int, std::pair<int, int>>> q;
    std::map<std::pair<int, int>, std::pair<int, int>> pre;
    
public:
    std::map<std::pair<int, int>, std::pair<int, int>> Findpath(std::pair<int,int>a,std::pair<int,int>b, Matrix &mat)
    { 
        pre.clear();
        for (int i = 0; i < 11; i++)
        {
            for (int j = 0; j < 11; j++)
            {
                g[i][j] = 0;
                v[i][j] = 0;
                // mp[i][j] = 0;
            }
        }
        
        // set the coordinate of the start and end point
        s.first = a.first;
        s.second = a.second;
        e.first = b.first;
        e.second = b.second;
        cout<<"The path start from: "<<s.first<<s.second<<endl;
        cout<<"The path end at: "<<e.first<<e.second<<endl;

        // pass the map in
        for (int i = 0; i < 11; i++)
        {
            for (int j = 0; j < 11; j++)
            {
                mp[i][j] = mat.Point(i, j);
            }
        }

        // Heuristic functions
        for (int i = 1; i < 10; i++)
        {
            for (int j = 1; j < 10; j++)
            {
                h[i][j] = abs(i - e.first) + abs(j - e.second);
            }
        }

        if (!mp[s.first][s.second])
        {
            q.push(std::make_pair(-(h[s.first][s.second] + g[s.first][s.second]), s));
            v[s.first][s.second] = 1;
        }

        while (!q.empty())
        {
            std::pair<int, int> tmp = q.top().second;
            q.pop();
            int x = tmp.first;
            int y = tmp.second;
            if (x == e.first && y == e.second)
            {
                break;
            }
            for (int i = 0; i <= 3; i++)
            {
                int xx = x + dx[i];
                int yy = y + dy[i];
                if (xx >= 1 && xx <=9 && yy >= 1 && yy <=9 && !v[xx][yy] && mp[xx][yy]!=1)
                {
                    v[xx][yy] = 1;
                    pre[std::make_pair(xx, yy)] = std::make_pair(x, y);
                    g[xx][yy] = g[x][y] + 1;
                    q.push(std::make_pair(-(h[xx][yy] + g[xx][yy]), std::make_pair(xx, yy)));
                }
            }
        }
        //return route of A*

        while(!q.empty())
      {
        q.pop();
      }

        return pre;
    }

};


#endif