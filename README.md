# Two Different Complete Coverage Path Planning Algorithms Comparation in Webots

The complete coverage path planning algorithm is a method that allows mobile robots to traverse each point in a known map environment. A classic application of this algorithm is a scene where a vacuum robot autonomously cleans an area. This article compared the efficiency of the Boustrophedon  Cell  Decomposition(BCD) algorithm and the Backtracking Spiral Algorithm (BSA) algorithm on the Webots simulation platform using E-puck autonomous mobile robot. The experiments are carried out in 5 kinds of grid map simulation environments. Experimental results show that it is more efficient to use Boustrophedon Cell Decomposition algorithm in most environments.

## Introduction:

The flow chart of CPP algorithm is shown below:

<center>

![ ](RS_Report/The%20Overall%20architecture%20of%20the%20CPP%20algorithm.png)

</center>

The results:

![ ](RS_Report/maps.png)
The experimential maps used in this project.


![ ](RS_Report/trajectorys.png)
The trajectory results for the maps.

[Video results](https://www.youtube.com/watch?v=0otNrg9-y_8&list=PLDFZ7pvu5Ykhd1IlsfWVeFoPCi_1xtch7&ab_channel=DAOMINGCHEN)

## Quick Access:
- [Robot_Workfolder](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder): Keeps the all developed fiels.
  - Controllers
    - [BSA](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder/controllers/BSA): BSA algorithm controller.
    - [Initial](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder/controllers/Initial): BCD algorithm controller.
  - [Worlds](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder/worlds): Keeps all the webots projects and world.
- [Results](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Results): Keeps all the experiment data .
- [Doc](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Doc): Keeps all the development log.
- [RS_Report](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/RS_Report): Keeps all the figures and the report in latex.

## Usage:

Before running the controller in webots, make sure to load the right map in main function.

![ ](RS_Report/load_map.png)

The 4 maps showed in introduction are namely:

![ ](RS_Report/4maps.png)

## Project Agenda:

> To view the graph below please add [GitHub + Mermaid extension](https://github.com/BackMarket/github-mermaid-extension) to your browser.

```mermaid
gantt

title Project Gantt
dateFormat YY-MM-DD

    section Stage 1
    Path Plan             :done, des2, 21-11-01, 3w
    Locomotion            :done, des2, 21-11-01, 3w
    Robot Pose            :done, des1, 21-11-01, 1w
    Mapping               :done, des1, 21-11-01, 2w
    Visulization          :done, des1, 21-11-01, 1w

    section Stage2
    Create Map                            :done, des3, after des1, 3d
    Change C to C++                       :done, des4, after des3, 5d  
    Study and Code Coverage Path Plan     :done, des5, after des4, 2w
    Study and Code A* Path Plan           :done, des5, after des4, 2w
    Debug & enhance system robustness     :done, des6, after des5, 1w
    
    section Stage3
    Collect data     :done, des7, after des6, 3d
    Create graphs    :done, des7, after des6, 3d

    section Stage4
    Documentaion                  :done, 21-11-01,7w
    Writing Paper                 :done, after des2, 4w

```

Ps:
- Stage 1: Decision-making stage
- Stage 2: Skill-building & API programming stage
- Stage 3: Experiment stage
- Stage 4: Documentation stage