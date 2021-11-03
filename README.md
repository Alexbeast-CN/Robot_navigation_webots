# Robot Navigation 

This project uses Webots as a simulator collaborating with Yifan Wang. The project is in progress.

## Project Agenda:

> To view the graph below please add [GitHub + Mermaid extension](https://github.com/BackMarket/github-mermaid-extension) to your browser.

```mermaid
gantt

title Project Gantt
dateFormat YY-MM-DD

    section Stage 1
    Path Plan             :active, des1, 21-11-01, 1w
    Locomotion            :active, des1, 21-11-01, 1w
    Odometry              :active, des1, 21-11-01, 1w
    Go Home               :active, des1, 21-11-01, 1w
    Visulization          :active, des1, 21-11-01, 1w

    section Stage2
    Cmake                 :des2, after des1, 1w
    Path Plan             :des2, after des1, 1w
    Locomotion            :des3, after des2, 1w

    section Stage3
    Documentaion          :active, 21-11-01,7w
    Paper                 :after des3, 4w

```

Ps:
- Stage 1: Decision-making stage
- Stage 2: Skill-building & API programming stage
- Stage 3: Documentation stage

## Project Index

- [View Source Code](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Controller)
- [Skill Notes](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/Doc/Useful%20Knowlege)
- [Formal Report](https://github.com/Alexbeast-CN/Robot_navigation_webots/tree/main/RS_Report)