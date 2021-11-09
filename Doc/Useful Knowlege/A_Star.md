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

> Logic Flow:
>1. S = {A};  U = {B,C,D,E,F}.<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C = 3;<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B = 6;<br>At this time, select C into S.
>2. S = {A,C};  U = {B,D,E,F}.<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B = 5;<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D = 6;<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">E = 7;<br>Because 5 is smaller than 6 written above,so
~~A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B~~ = A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B<br>Since 5 is the minimum distance at this stage, adding B to S.
>3. S = {A,C,B};  U = {D,E,F}.<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">B<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D = 10<br>Since 10 is greater than 6, so this route should not be used, but A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D  should be used.<br>After the comparison, add the compared D points to the set S.
>4. S = {A,C,B,D};  U = {E,F}.<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">E = 8;<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">D<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">F = 9;<br>Since the value of 8 is greater than 7, the shortest route to point E should be A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">E,.
>5. S = {A,C,B,D,E};  U = {F}.<br>A<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">C<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">E<img src="https://render.githubusercontent.com/render/math?math=\rightarrow">F = 12;<br> Because this value is too large, the route to point F will not adopt this method, but the above method.

So far, we have found the shortest path from point A to each of these points.

## 2. A* Algorithm
This section gives a brief introduction to the algorithm by combining it with visual code.<br>
This section requires some knowledge of C++，which includes functions for data structures and data processing in STL, such as map, pair and priority queue.
