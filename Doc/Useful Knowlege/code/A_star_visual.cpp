#include<iostream>
#include<queue>
#include<utility>
#include<cmath>
#include<cstdio>
#include<map>
#include<windows.h>
#define mp(x,y) make_pair(x,y)
 
using namespace std;
 
//输入：行数，列数，图形，起点，终点
//如下输入 ↓↓ 
/* 
6 6
0 0 0 0 0 0
0 1 1 1 1 0
0 0 0 1 0 0
1 1 0 1 1 0
0 0 0 1 1 0
1 1 0 0 0 0
3 1
3 6
*/
 
int n,m;
 
int mp[25][25];
 
char graph[25][25];
 
int h[25][25],g[25][25];
 
int v[25][25];
 
pair<int,int> s,e;
 
priority_queue< pair< int,pair<int,int> > > q;
 
int dx[4]={-1,0,1,0};
int dy[4]={0,1,0,-1};
 
int flag=0;
 
map< pair<int,int>,pair<int,int> > pre;//记录前驱节点 
 
void print_graph(){//打印棋盘 
	system("cls");
	for(int i=1;i<=3;i++)cout<<"\n";
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=3;j++)cout<<"\t";
		for(int j=1;j<=m;j++)
		{
			cout<<graph[i][j]<<" ";
		}
		cout<<"\n";
	}
	Sleep(1000);
}
 
void move(pair<int,int> x){//棋子移动 
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			if(graph[i][j]=='*')
			{
				graph[i][j]='*';
				break;
			}
		}
	}
	graph[x.first][x.second]='*';
}
 
void print(pair<int,int> x){//递归输出 
	if(x==s)
	{
		move(x);
		print_graph();
	}else{
		print(pre[x]);
		move(x);
		print_graph();
	}
}
 
inline int read(){//快速读入
	int ans0=0;
	char ch=getchar();
	while(!isdigit(ch))
	{
		ch=getchar();
	}
	while(isdigit(ch))
	{
		ans0=ans0*10+ch-'0'; 
		ch=getchar();
	}
	return ans0;
}
 
int main()
{
	n=read(),m=read();
	
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			mp[i][j]=read();
		}
	}
	
	cin>>s.first>>s.second>>e.first>>e.second;
	
	//建图 
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			graph[i][j]=mp[i][j]+'0';
		}
	}
	
	graph[s.first][s.second]='*';
	
	//构造启发函数为该节点到终点的曼哈顿距离 
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			h[i][j]=abs(i-e.first)+abs(j-e.second);
		}
	}
 
	//广搜，优先队列优化 
	if(!mp[s.first][s.second])
	{
		q.push(mp(-(h[s.first][s.second]+g[s.first][s.second]),s));
		v[s.first][s.second]=1;	
	} 
	
	while(!q.empty())
	{
		pair<int,int> tmp=q.top().second;
		q.pop();
		int x=tmp.first,y=tmp.second;
		if(x==e.first&&y==e.second)
		{
			flag=1;
			break;
		}
		for(int i=0;i<=3;i++)
		{
			int xx=x+dx[i];
			int yy=y+dy[i];
			if(xx>=1&&xx<=n&&yy>=1&&yy<=m&&!v[xx][yy]&&!mp[xx][yy])
			{
				v[xx][yy]=1;
				pre[mp(xx,yy)]=mp(x,y);
				g[xx][yy]=g[x][y]+1; 
				q.push(mp(-(h[xx][yy]+g[xx][yy]),mp(xx,yy)));
			}
		}
	}
	
	if(!flag)cout<<"抱歉，找不到所求路径";
	else{
		print(e);
	}
	
	return 0;
}
