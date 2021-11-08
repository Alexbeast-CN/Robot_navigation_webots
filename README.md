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
    Path Plan             :active, des2, 21-11-01, 3w
    Locomotion            :active, des2, 21-11-01, 3w
    Odometry              :active, des1, 21-11-01, 1w
    Go Home               :active, des2, 21-11-01, 3w
    Visulization          :active, des1, 21-11-01, 1w

    section Stage2
    Cmake                 :des3, after des1, 1w
    Create Map            :des3, after des1, 3d
    Change C to C++       :des4, after des3, 10d

    Coverage Path Plan    :des4, after des3, 2w
    Locomotion            :des5, after des2, 1w

    section Stage3
    Documentaion          :active, 21-11-01,7w
    Paper                 :after des2, 4w

```

Ps:
- Stage 1: Decision-making stage
- Stage 2: Skill-building & API programming stage
- Stage 3: Documentation stage

## Quick Access

- [View Source Code](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Controller)
- [Notes](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Doc/Useful%20Knowlege)
- [Formal Report](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/RS_Report)