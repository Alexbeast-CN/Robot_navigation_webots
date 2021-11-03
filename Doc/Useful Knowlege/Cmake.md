# Notes2Cmake

> This is a lightweight note to Cmake. Only KEY functions. I didn't ref the official tutorial because it's too heavy.

> ref: 
> - [video Tutorial of Cmake](https://www.youtube.com/playlist?list=PLalVdRk2RC6o5GHu618ARWh0VO0bFlif4)
> - [Blog on Zhihu](https://zhuanlan.zhihu.com/p/119426899)

## 1. Essentials for a project

```c
cmake_minimum_required(VERSION 3.10) 

# set the project name 
project(Name VERSION 1.0)

# add the executable 
add_executable(${PROJECT_NAME} main.cpp)
```