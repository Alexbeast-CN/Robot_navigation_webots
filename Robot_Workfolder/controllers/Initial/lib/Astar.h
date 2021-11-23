#ifndef ASTAR_H
#define ASTAR_H

#include<iostream>
#include<cmath>
#include<queue>
#include<utility>
#include<cstdio>
#include<map>

class Astar
{
private:
    int h[11][11],g[11][11],v[11][11];
    int dx[4] = {-1,0,1,0};
    int dy[4] = {0,1,0,-1};
    std::pair<int,int> s,e;//起始点和终点坐标
    // 优先队列比较启发函数，并储存父节点
    std::priority_queue<std::pair< int,std::pair<int,int> > > q;
    // 生成记录前期位置的容器
    std::map<std::pair<int,int>,std::pair<int,int> > pre;
public:
    Astar()
    {
        
    }
    
    
};

Astar::Astar(/* args */)
{
}

Astar::~Astar()
{
}

#endif