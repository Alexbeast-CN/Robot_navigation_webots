#ifndef ASTAR_H
#define ASTAR_H

#include<iostream>
#include<cmath>
#include<queue>
#include<utility>
#include<cstdio>
#include<map>

#include "Matrix.h"
#include "Map.h"

#define mp(x,y) std::make_pair(x,y)

class Astar
{
private:
    int mp[11][11],h[11][11],g[11][11],v[11][11];
    int dx[4] = {-1,0,1,0};
    int dy[4] = {0,1,0,-1};
    std::pair<int,int> s,e;//起始点和终点坐标
    // 优先队列比较启发函数，并储存父节点
    std::priority_queue<std::pair< int,std::pair<int,int> > > q;
    // 生成记录前期位置的容器
    std::map<std::pair<int,int>,std::pair<int,int> > pre;

    
public:
    int Findpath(int a[2], int b[2], Matrix mat)
    {   
        // 将形参数组中的值传递给pair变量，其为起点和终点坐标
        s.first = a[0];
        s.second = a[1];
        e.first = b[0];
        e.second = b[1];

        // 将形参地图传给实参地图
        for(int i=0; i<11; i++)
        {
            for(int j=0; j<11; j++)
            {
                mp[i][j] = mat.p[i][j];
            }
        }
        // 构造启发函数为该节点到终点的曼哈顿距离
        for(int i=1;i<=11;i++)
	    {
		    for(int j=1;j<=11;j++)
		    {
			    h[i][j]= abs(i-e.first)+ abs(j-e.second);
		    }
	    }
        //优先队列优先的搜索方法
        if(!mp[s.first][s.second])
	    {
		    q.push(mp(-(h[s.first][s.second]+g[s.first][s.second]),s));
		    v[s.first][s.second]=1;	
	    } 
	
	    while(!q.empty())
	    {
		    std::pair<int,int>tmp = q.top().second;
		    q.pop();
		    int x = tmp.first; 
            int y = tmp.second;
		    if(x==e.first && y==e.second)
		    {
			    break;
		    }
		    for(int i=0;i<=3;i++)
		    {
			    int xx = x + dx[i];
			    int yy = y + dy[i];
			    if(xx>=1 && xx<=11 && yy>=1&& yy<=11 &&!v[xx][yy] && !mp[xx][yy])
			    {   
				    v[xx][yy]=1;
				    pre[mp(xx,yy)]=mp(x,y);
				    g[xx][yy]=g[x][y]+1; 
				    q.push(mp(-(h[xx][yy]+g[xx][yy]),mp(xx,yy)));
			    }
		    }
	    }
    }

    Astar()
    {
        //将g和v的两个二维数组初始化为0
        for(int i=0; i<11; i++)
            for(int j=0; j<11; j++)
            {
                g[i][j] = 0;
                v[i][j] = 0;
            }
    }

    
};

Astar::Astar(/* args */)
{
}

Astar::~Astar()
{
}

#endif