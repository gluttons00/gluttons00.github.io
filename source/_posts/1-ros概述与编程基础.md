---
title: 1 ros概述与编程基础
tags: [ros]
cover: ./images/封面-ros.png
date: 2024-05-02 10:51:17
categories: ros
---

# ros程序实现流程

1. 建立工作空间

   ```powershell
   mkdir -p 工作空间名/src #创建多级目录
   rm -r 工作空间名/src #删除多级目录
   ```


2. 进入工作空间并编译

   ```powershell
   cd 工作空间名
   catkin_make
   ```

3. 建立功能包、设置依赖

   ```powershell
   #方法1
   catkin_create_pkg 功能包名 依赖1 依赖2 ···
   #方法2
   在vscode中可以直接右键工作空间下的 src 选择建立功能包
   ```

4. 在功能包的src目录下创建 文件名.cpp 文件

5. 编写源文件

   （1）包含头文件

   （2）main()

   ```c++
   int main(int argc, char* argv[]){
       ···
   	return 0;
   }
   ```

   （3）初始化节点

   `ros::init(argc, argv, "节点名");`

   （4）创建句柄

   `ros::NodeHandle nh; //nh为句柄名`

   （5）其他代码，功能代码

   ```c++
   //设置输出文字格式（解决文字异常）
   setlocale(LC_ALL,"");
   ```

6. 配置 CMakeList.txt

   ```cmake
   add_executable(映射名
     src/源文件名.cpp
   )
   target_link_libraries(映射名
     ${catkin_LIBRARIES}
   )
   ```

7. 新建终端，启动核心：`roscore`

8. 新建终端，运行ROS代码：

   ```powershell
   #通过 source 命令来朱行执行setup.bash的命令，来修改环境变量
   source ./devel/setup.bash
   #运行ros程序
   rosrun 功能包名 映射名
   ```

# 附录

1. "::"在c++中为：作用域分辨符（调用自定义头文件类函数）

2. source 命令：顺序执行文件中的命令

3. echo 命令：ubuntu输出命令（可利用 >> 重定向输出到文件中）

4. `NodeHandle`：是ROS的一个重要组件，提供一个用于在自定义程序和ROS系统之间通信的接口

5. `catkin`：是ROS基于 CMake 的编译构建系统，解决了多软件包构建顺序问题，可扩阿姨来项目编译，自动生成配置文件

6. 在 vscode 中可用

   （1）ctrl + "+"：放大界面

   （2）crtl + shift + B 来快捷编译选择 catkin_make : build （需配置 tasks.json ）

   （3）配置c_cpp_properties.json的 includePath 加上"/home/用户名/……/工作空间/src/功能包/include/**"

   （4）ctrl + "`"打开终端

# launch文件的多节点启动

1. 在功能包上创建 launch 文件夹并创建 .launch 文件

2. 编写 .launch 文件

   ```html
   <launch>
       <node pkg = "功能包名" type = "节点文件名" name = "映射名" output = "screen" />
   	<!--output属性为：日志输出目标（可无）-->
           ···
       </node>
   </launch>
   ```
   
3. 运行

   （1）`source ./devel/setup.bash`

   （2）`roslaunch 功能包名 文件名.launch`

