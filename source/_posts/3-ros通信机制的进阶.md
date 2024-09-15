---
title: 3 ros通信机制的进阶
tags: [ros]
cover: ./images/封面-ros.png
date: 2024-05-04 12:58:38
categories: ros
---

# 3.1 常用API

**API（Application Programming Interface,应用程序编程接口）：**是一些预先定义的函数，目的是提供应用程序与开发人员基于某软件或硬件得以访问一组例程的能力，而又无需访问源码，或理解内部工作机制的细节

## 初始化

ros::init(argc, argv, name, options)

**参数：**

```cpp
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);

/** @brief ROS初始化函数。
 *
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
 *
 * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
```

**使用：**

1. argc、argv

   按照特定的格式传参，可以加以利用，如设置全局参数：`rosrun 包名 文件名 _变量名 := xx`

2. options（同节点的多次使用）

   `ros::init(aergc, argv, "xx", ros::init_options::AnonymousName);`

## 时间

ROS中时间相关的API是极其常用，比如:获取当前时刻、持续时间的设置、执行频率、休眠、定时器...都与时间相关。

### 时刻

获取时刻，或是设置指定时刻:

1. 获取当前时刻 `ros::Time xx = ros::Time::now();`

   ```c++
   ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
   ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
   
   //获取距离 1970年01月01日 00:00:00 的秒数
   right_now.toSec() //double型
   right_now.sec //int型
   ```

   

2. 设置指定时刻`ros::Time xx(···);`

   ```c++
   ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
   ROS_INFO("时刻:%.2f", someTime.toSec()); //100.10
   
   ros::Time someTime2(100.3);//直接传入 double 类型的秒数
   ROS_INFO("时刻:%.2f", someTime2.toSec()); //100.30
   ```

   

### 2.持续时间

设置一个时间区间(间隔):`ros::Duration xx(10).sleep();`

```cpp
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());

ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位
du.sleep();//按照指定的持续时间休眠

ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
```



### 3.持续时间与时刻运算

为了方便使用，ROS中提供了时间与时刻的运算:

```cpp
ROS_INFO("时间运算");
ros::Time now = ros::Time::now();
ros::Duration du1(10);
ros::Duration du2(20);
ROS_INFO("当前时刻:%.2f",now.toSec());

//1.time 与 duration 运算
ros::Time after_now = now + du1;
ros::Time before_now = now - du1;
ROS_INFO("当前时刻之后:%.2f",after_now.toSec());
ROS_INFO("当前时刻之前:%.2f",before_now.toSec());

//2.duration 之间相互运算
ros::Duration du3 = du1 + du2;
ros::Duration du4 = du1 - du2;
ROS_INFO("du3 = %.2f",du3.toSec());
ROS_INFO("du4 = %.2f",du4.toSec());
//PS: time 与 time 不可以运算
// ros::Time nn = now + before_now;//异常

//3.time 与 time 运算
ros::Duration du = before_now - after_now;
```



### 4.设置运行频率

```cpp
ros::Rate rate(1);//指定频率
while (true)
{
    ROS_INFO("-----------code----------");
    rate.sleep();//休眠，休眠时间 = 1 / 频率。
}
```



### 5.定时器

ROS 中内置了专门的定时器，可以实现与 ros::Rate 类似的效果:

```cpp
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
ros::Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false, bool autostart = true);
/**
* \brief 创建一个定时器，按照指定频率调用回调函数。
*
* \param period 时间间隔
* \param callback 回调函数
* \param oneshot 如果设置为 true,只执行一次回调函数，设置为 false,就循环执行
* \param autostart 如果为true，返回已经启动的定时器,设置为 false，需要通过timer.start()手动启动
*/

 // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing);
 ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,true);//只执行一次
 ros::spin(); //必须 spin
```

定时器的回调函数:

```cpp
void doSomeThing(const ros::TimerEvent &event){
    ROS_INFO("-------------");
    ROS_INFO("event:%s",std::to_string(event.current_real.toSec()).c_str());
}
```

## 其他函数

1.节点状态判断

```cpp
/** \brief 检查节点是否已经退出
 *
 *  ros::shutdown() 被调用且执行完毕后，该函数将会返回 false
 *
 * \return true 如果节点还健在, false 如果节点已经火化了。
 */
bool ok();
```

2.节点关闭函数

```cpp
/*
*   关闭节点
*/
void shutdown();
```

3.日志函数

```cpp
ROS_DEBUG("hello,DEBUG"); //不会输出
ROS_INFO("hello,INFO"); //默认白色字体
ROS_WARN("Hello,WARN"); //默认黄色字体
ROS_ERROR("hello,ERROR");//默认红色字体
ROS_FATAL("hello,FATAL");//默认红色字体
```

# 3.2 ROS中的头文件与源文件

## 3.2.1 编写头文件

在功能包下的 include 目录下新建头文件: haha.h

```c++
//使用ifndef,endif来限定宏定义范围
#ifndef _HAHA_H //头文件名
#define _HAHA_H

/*
namespace:
    为了帮助编译器判断不同库中的同名函数，引入了命名空间这个概念，它可作为附加信息来区分不同库中相同名称的函数、类、变量等。使用了命名空间即定义了上下文。本质上，命名空间就是定义了一个范围配合作用域分辨符"::"，即可访问命名空间中的各个类。类使用只需要创建对象后，通过"类.成员"即可访问。
*/
namespace hello_ns { 
    
    class My {
        
        public: 
        	void run(); //声明类 My 的函数 run()，在源文件中编写具体内容
        	……
    };
}
#endif
```

在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c_cpp_properties.json 的 includepath属性，注意要以 `/include/**`结尾

```powershell
"/home/用户/工作空间/src/功能包/include/**"
```



## 3.2.2 编写源文件

在 /工作空间/src/功能包/src 目录下新建源文件:haha.cpp

```C++
#include "test_head_src/haha.h"
#include "ros/ros.h"

namespace hello_ns{
    
    void My::run(){
        ROS_INFO("hello,head and src ...");
    }
}
```

## 3.2.3 编写可执行文件

在 /工作空间/src/功能包/src 目录下新建文件: use_head.cpp

```C++
#include "ros/ros.h"
#include "test_head_src/haha.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hahah");
    hello_ns::My my;
    my.run();
    return 0;
}

```



## 3.2.4 编辑配置文件并执行

头文件与源文件相关配置：

```cmake
#取消第2行#include的注释，指出添加的头文件位置（估计是为了方便编译），你的功能包位置，要在其他位置之前。
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## 声明C++库
add_library(head # 库的映射名
  include/test_head_src/haha.h
  src/haha.cpp
)

# 此处函数的第一个参数也是库的映射名，需要与add_library的相同
add_dependencies(head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head
  ${catkin_LIBRARIES}
)

```

可执行文件配置：

```cmake
add_executable(use_head src/use_head.cpp)

# 与配置库函数的add_dependencies相同但是位置不同，为提高可读性不要乱写
add_dependencies(use_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#此处需要添加之前设置的 head 库
target_link_libraries(use_head
  head
  ${catkin_LIBRARIES}
)
```

