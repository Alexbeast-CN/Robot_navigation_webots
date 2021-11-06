# A* Algorithm
>This note is a brief introduction to the A* algorithm and provides an introduction to the visualization of the algorithm.

>Ref:
> - [Algorithm Detail of A*](https://www.cnblogs.com/21207-iHome/p/6048969.html#undefined)
> - [Visualization Algorithm](https://github.com/redglassli/PythonRobotics)

## 1. Dijkstra Algorithm
The algorithm is a shortest path algorithm, the logic of which is to find the closest point to the point at a time and traverse all the points in the graph.

> Algorithm steps: 
>1. Set two sets S and U.Initially, S contains only the origin and the rest of the points are stored in U.
>2. Select the point from U with the smallest distance from the starting point and add this point to S.
>3. Use the above point as a new starting point and repeat the above steps.

A simple example is given in the following figure：(Start with A)

![Dijkstra](https://pic002.cnblogs.com/images/2012/426620/2012073019593375.jpg#pic_center)

1. S = {A};  U = {B,C,D,E,F}. 
A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C = 3;
A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B = 6; 
At this time, select C into S.
2. S = {A,C};  U = {B,D,E,F}.
A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B = 5;
A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D = 6;
A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">E = 7;
Because 5 is smaller than 6 written above,
~~A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B~~ = A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B
Since 5 is the minimum distance at this stage, adding B to S.
3. S = {A,C};  U = {B,D,E,F}.

