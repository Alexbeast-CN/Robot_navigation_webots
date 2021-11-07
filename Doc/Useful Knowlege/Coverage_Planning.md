# Coverage Planning 

> Book: 
> - [Planning Algorithm -- Steven M. LaValle](http://lavalle.pl/planning/)
>   - [Boustrophedon decomposition](http://lavalle.pl/planning/node352.html)
>   - [Spanning tree covering](http://lavalle.pl/planning/node353.html)

The coverage planning algorithm here is the main to solve problems in 2D areas. There are two methods explained in this book.

## 1. Boustrophedon decomposition 

> ref: [Convex Decomposition for a Coverage Path Planning for Autonomous Vehicles: Interior Extension of Edges](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6806237/)
### 1.1 Full Coverage Path Plan

Boustrophedon means the way ox walks which is the parallel-line-covered areas. The pattern is illustrated in the figure below. 

![ ](./pics/sensors-19-04165-g001.jpg)

When the map is a convex polygon, Boustrophedon is easy to be applied in a coverage task. The leading optimization in this part is to reduce turning, as turning costs more energy and time than moving straight. 

![ ](./pics/sensors-19-04165-g002.jpg)

However, when the area is complex and concave, the difficulty levels up. One approach for a complex map is to decompose it into convex subareas. Then apply boustrophedon to each sub-area and connect this coverage path sub-area to complete the entire coverage path plan. The connection method is formulated as a Traveling Salesman Problem. What we need to do is to minimize the times the robot visit all the sub-area.

![ ](./pics/sensors-19-04165-g004.jpg)

### 1.2 Decomposition method 

It's not hard for a robot to plan a path in a sub-polygon. However, the difficulty is how to segment an area into sub-areas and achieve minimum turning. 

A solution called [Interior Extension of Edges](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6806237/). It starts by determining if the internal angle between two edges of a target polygon is more than 180 degrees. Then extend the edges of those picked angles until they hit the boundaries of the map, as illustrated below.

![ ](pics/sensors-19-04165-g008.jpg)

After that, the neighbor sub-polygons need to be merged to reduce sub-areas. The principle of this process the merged polygons are convex. In this process, we can get a set of merge options. Among them, The best option is the one with minimum total width.   

![ ](pics/sensors-19-04165-g009.jpg)

### 1.3 Method Detail

> ref: [A phd dissertation of Coverage Planning Problem and Traveling Salesman Problems](https://etd.auburn.edu/bitstream/handle/10415/4599/Optimization%20Approaches%20for%20a%20Dubins%20Vehicle%20in%20Coverage%20Planning%20Problem%20and%20Traveling%20Salesman%20Problems.pdf?sequence=2)

### 1.4 Code

> ref: [RicheyHuang/CoveragePlanner](https://github.com/RicheyHuang/CoveragePlanner)