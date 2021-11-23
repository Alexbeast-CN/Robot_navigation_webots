# Robot Navigation 

This project uses Webots as a simulator to test path planning algorithms for an indoor robot.

The project is in progress.

## Project Agenda:

> To view the graph below please add [GitHub + Mermaid extension](https://github.com/BackMarket/github-mermaid-extension) to your browser.

```mermaid
gantt

title Project Gantt
dateFormat YY-MM-DD

    section Stage 1
    Path Plan             :done, des2, 21-11-01, 3w
    Locomotion            :active, des2, 21-11-01, 3w
    Odometry              :active, des1, 21-11-01, 1w
    Go Home               :active, des2, 21-11-01, 3w
    Mapping               :active, des1, 21-11-01, 2w
    Visulization          :active, des1, 21-11-01, 1w

    section Stage2
    Cmake                 :active, des6, after des1, 1w
    Create Map            :done, des3, after des1, 3d
    Study Mapping         :active, des4, after des3, 10d

    Change C to C++       :active, des5, after des6, 10d
    
    Study and Code Coverage Path Plan    :active, des5, after des6, 3w

    Study and Code A* Path Plan    :active, des5, after des6, 3w
    

    section Stage3
    Documentaion          :active, 21-11-01,7w
    Writing Paper                 :after des2, 4w

```

Ps:
- Stage 1: Decision-making stage
- Stage 2: Skill-building & API programming stage
- Stage 3: Documentation stage

## Quick Access

- [Webots Project](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfile)
  - Controller
    - [Main](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder/controllers/Initial)
    - [Lib](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Robot_Workfolder/controllers/Initial/lib)
- [Documentation](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Doc/Useful%20Knowlege)
- [Formal Report](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/RS_Report)