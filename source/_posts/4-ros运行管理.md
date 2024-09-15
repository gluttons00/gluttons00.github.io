---
title: 4 ros运行管理
tags: [ros]
cover: ./images/封面-ros.png
date: 2024-05-07 14:23:34
categories: ros
---

​	一个完整的ros系统，可能包括多台主机可能包含多台主机；每台主机上又有多个工作空间(workspace)；每个的工作空间中又包含多个功能包(package)；每个功能包又包含多个节点(Node)，不同的节点都有自己的节点名称；每个节点可能还会设置一个或多个话题(topic)，因此我们需要对ros的运行进行管理。

---

# 4.1 ROS元功能包

​	元功能包（Meta Package）是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

​	作用：提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个复杂功能模块时，直接调用打包后的功能包即可

​	用法：

1. 新建一个功能包，与需集成的功能包在同一工作空间，无任何依赖（元功能包是ROS中的一个虚包）

2. 修改 **package.xml** ,内容如下:

```xml
 <exec_depend>被集成的功能包</exec_depend>
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
```



3. 修改 **CMakeLists.txt** ,内容如下:

```cmake
#前三行 CMakeLists 原本就存在
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
#第四行需要填上
catkin_metapackage()
```

---

# 4.2 ROS节点运行管理launch文件

launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数。

作用：简化节点的配置与启动，提高ROS程序的启动效率。

1. 新建launch文件：在功能包下添加 launch目录, 目录下新建 xxxx.launch 文件，编辑 launch 文件

   * 含子标签开头<--->

   * 含子标签结尾</--->
   * 不含子标签<---/>
   * 子标签注意退格（Tab）

```xml
<launch>
	<node pkg="功能包名" type="节点文件名" name="映射名" output="日志输出目标（可省略）" />
    <node pkg="turtlesim" type="turtlesim_node" name="myTurtle" output="screen" />
</launch>
```

2. 调用 launch 文件

```powershell
source ./devel/setup.bash
roslaunch 包名 文件名.launch
```

**注意:** roslaunch 命令执行launch文件时, 首先会判断是否启动了 roscore , 如果启动了, 则不再启动, 否则, 会自动调用 roscore



## launch文件标签之 launch

`<launch>`标签是所有 launch 文件的根标签，充当其他标签的容器

1. 属性

- `deprecated = "弃用声明"` 告知用户当前 launch 文件已经弃用

2. 子级标签

- 所有其它标签都是launch的子级

## launch文件标签之 node

`<node>`标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)

1. 属性

- pkg="功能包名"

  节点所属的包

- type="节点文件名"

  节点类型(与之相同名称的可执行文件)

- name="映射名"

  节点名称(在 ROS 网络拓扑中节点的名称)

- args="xxx xxx xxx" (可选)

  将参数传递给节点

- machine="机器名"

  在指定机器上启动节点

- respawn="true | false" (可选)

  如果节点退出，是否自动重启

- respawn_delay=" N" (可选)

  如果 respawn 为 true, 那么延迟 N 秒后启动节点

- required="true | false" (可选)

  该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch

- ns="xxx" (可选)

  在指定命名空间 xxx 中启动节点

- clear_params="true | false" (可选)

  在启动前，删除节点的私有空间的所有参数

- output="log | screen" (可选)

  日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log

2. 子级标签

- env 环境变量设置
- remap 重映射节点名称
- rosparam 参数设置
- param 参数设置

## launch文件标签之 include

`include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

1. 属性

- file="$(find 包名)/xxx/xxx.launch"（要包含的文件路径）

- ns="xxx" (可选)

  在指定命名空间导入文件

2. 子级标签

- env 环境变量设置
- arg 将参数传递给被包含的文件

## launch文件标签之 remap

用于话题重命名

1. 属性

- from="xxx" 原始话题名称
- to="yyy" 目标名称

## launch文件标签之 param

`<param>`标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。

1. 属性

- name="命名空间/参数名"

  参数名称，可以包含命名空间

- value="xxx" (可选)

  定义参数值，如果此处省略，必须指定外部文件作为参数源

- type="str | int | double | bool | yaml" (可选)

  指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:

  - 如果包含 '.' 的数字解析为浮点型，否则为整型
  - "true" 和 "false" 是 bool 值(不区分大小写)
  - 其他是字符串

## launch文件标签之rosparam

`<rosparam>`标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，`<rosparam>`标签在`<node>`标签中时被视为私有。

1. 属性

- command="load | dump | delete" (可选，默认 load)

  加载、导出或删除参数

- file="$(find xxxxx)/xxx/yyy...."

  加载或导出到的 yaml 文件

- param="参数名称"

- ns="命名空间" (可选)

## launch文件标签之group

`<group>`标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间

1. 属性

- ns="名称空间" (可选)

- clear_params="true | false" (可选)

  启动前，是否删除组名称空间的所有参数(慎用....此功能危险)

2. 子级标签

- 除了launch 标签外的其他标签

## launch文件标签之arg

`<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

1. 属性

- name="参数名称"

- default="默认值" (可选)

- value="数值" (可选)

  不可以与 default 并存

- doc="描述"

  参数说明

2. 示例

- launch文件传参语法实现,hello.lcaunch

  ```xml
  <launch>
      <arg name="xxx" />
      <param name="param" value="$(arg xxx)" />
  </launch>
  ```
  
- 命令行调用launch传参

  ```powershell
  roslaunch hello.launch xxx:=值
  ```

---

# 4.3 ROS节点名称重名

所谓ROS节点名称重名，就是同一个节点文件的在同一时间段内的多次使用，造成的节点名冲突

解决策略：

- 使用命名空间：就是为名称添加前缀
- 名称重映射：为名称起别名

## rosrun 设置命名空间与重映射

**1.命名空间：**

语法: rosrun 功能包名 节点名 __ns:=新名称

```powershell
rosrun turtlesim turtlesim_node __ns:=/xxx
rosrun turtlesim turtlesim_node __ns:=/yyy
```

`rosnode list`查看节点信息,显示结果:

```powershell
/xxx/turtlesim
/yyy/turtlesim
```

**2.重映射：**

语法: rosrun 包名 节点名 __name:=新名称

```powershell
rosrun turtlesim  turtlesim_node __name:=t1 #任意一种即可
rosrun turtlesim  turtlesim_node /turtlesim:=t1

rosrun turtlesim  turtlesim_node __name:=t2
rosrun turtlesim  turtlesim_node /turtlesim:=t2
```

`rosnode list`查看节点信息,显示结果:

```powershell
/t1
/t2
```

**设置命名空间同时名称重映射**

语法: rosrun 包名 节点名 __ns:=新名称 __name:=新名称

```powershell
rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
```

## launch文件设置命名空间与重映射

**1.launch文件**

```xml
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="turtlesim" type="turtlesim_node" name="t2" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>

</launch>
```

在 node 标签中，name 属性是必须的，ns 可选。

**2.运行**

`rosnode list`查看节点信息,显示结果:

```powershell
/t1
/t2
/t1/hello
```

## 编码设置命名空间与重映射

**重映射**

核心代码:

```c++
ros::init(argc,argv,"zhangsan",ros::init_options::AnonymousName);
//会在名称后面添加时间戳。
```



**命名空间**

核心代码

```c++
std::map<std::string, std::string> map;
map["__ns"] = "xxxx";
ros::init(map,"wangqiang"); // 不再传递argc，argv而是传递一个map集合节点名称
//节点名称设置了命名空间。
```

---

# 4.4 ROS话题名称设置

在实际应用中，按照逻辑，有些时候可能需要将相同的话题名称设置为不同，也有可能将不同的话题名设置为相同。在ROS中给出的解决策略也是使用**名称重映射或为名称添加前缀**。根据前缀不同，有全局、相对、和私有三种类型之分。

- 全局(参数名称直接参考ROS系统，与节点命名空间平级)
- 相对(参数名称参考的是节点的命名空间，与节点名称平级)
- 私有(参数名称参考节点名称，是节点名称的子级)

## rosrun设置话题重映射

*rosrun名称重映射语法: rorun 包名 节点名 话题名:=新话题名称**

将 teleop_twist_keyboard 节点的话题设置为`/turtle1/cmd_vel`

```powershell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel
rosrun 功能包名				  节点名                    话题名 := 新话题名
```



## launch文件设置话题重映射

*launch 文件设置话题重映射语法:**

```xml
<node pkg="xxx" type="xxx" name="xxx">
    <remap from="原话题" to="新话题" />
</node>
```



## 编码设置话题名称

话题的名称与节点的命名空间、节点的名称是有一定关系的，话题名称大致可以分为三种类型:

- 全局(话题参考ROS系统，与节点命名空间平级)
- 相对(话题参考的是节点的命名空间，与节点名称平级)
- 私有(话题参考节点名称，是节点名称的子级)

结合编码演示具体关系。

### 1. 全局名称

**格式:以`/`开头的名称，和节点名称无关**

比如：`/参数名称`

示例1:`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter",1000);`

结果1**:**`/chatter`

示例2:`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);`

结果2:`/chatter/money`

### 2. 相对名称

**格式:非`/`开头的名称,参考命名空间(与节点名称平级)来确定话题名称**

示例1:`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);`

结果1:`/命名空间/chatter`

示例2:`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money",1000);`

结果2:`/命名空间/chatter/money`

### 3. 私有名称

**格式:以`~`开头的名称**

示例1:

```c++
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);
```

结果1:`/节点名称/chatter` 或 `/命名空间/节点名称/chatter`

*PS:当使用*`~`*,而话题名称有时*`/`*开头时，那么话题名称是绝对的*

示例2:

```c++
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);
```

结果2:`/chatter/money`

# 4.5 ROS参数名称设置

在ROS中节点中，当参数名称重名时，会产生覆盖，如何避免。

关于参数重名的处理，**没有重映射实现**，为了尽量的避免参数重名，都是使用**为参数名添加前缀**的方式，实现类似于话题名称，有全局、相对、和私有三种类型之分。

- 全局(参数名称直接参考ROS系统，与节点命名空间平级)
- 相对(参数名称参考的是节点的命名空间，与节点名称平级)
- 私有(参数名称参考节点名称，是节点名称的子级)

设置参数的方式也有三种:

- rosrun 命令
- launch 文件
- 编码实现

## rosrun设置参数

rosrun 在启动节点时，也可以设置参数:

**私有模式，参考的是 /命名空间(功能包名)/参数名称**

**语法:** rosrun 包名 节点名称 _参数名:=参数值

1. 设置参数

   启动乌龟显示节点，并设置参数 _A = 100

   ```powershell
   rosrun turtlesim turtlesim_node _A:=100
   ```

2. 运行结果

   `rosparam list`查看节点信息,显示结果:

   ```powershell
   /turtlesim/A #有节点名称前缀，所以为私有模式，即该节点私有，（turtlesim_node的名字就是turtlesim）
   ```

## launch文件设置参数

通过 launch 文件设置参数的方式前面已经介绍过了，可以在 node 标签外，或 node 标签中通过 param 或 rosparam 来设置参数。在 **node 标签外设置的参数是全局性质的**，参考的是 **/** ，**在 node 标签中设置的参数是私有性质的**，参考的是 **/节点名称/参数名称**。

1. 设置参数

   以 param 标签为例，设置参数

   ```xml
   <launch>
   
       <param name="p1" value="100" />
       <node pkg="turtlesim" type="turtlesim_node" name="t1">
           <param name="p2" value="100" />
       </node>
   
   </launch>
   ```

2. 运行

   `rosparam list`查看节点信息,显示结果:

   ```powershell
   /p1
   /t1/p2
   ```

##  编码设置参数

编码的方式可以更方便的设置:全局、相对与私有参数。

### 1. ros::param设置参数

设置参数调用API是ros::param::set，该函数中，参数1传入参数名称，参数2是传入参数值，参数1中参数名称设置时，如果以 / 开头，那么就是全局参数，如果以 ~ 开头，那么就是私有参数，既不以 / 也不以 ~ 开头，那么就是相对参数。代码示例:

```c++
ros::param::set("/set_A",100); //全局,和命名空间以及节点名称无关
ros::param::set("set_B",100); //相对,参考命名空间
ros::param::set("~set_C",100); //私有,参考命名空间与节点名称
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```powershell
/set_A
/xxx/set_B
/xxx/yyy/set_C
```



### 2. ros::NodeHandle设置参数

如果参数名以 / 开头，那么就是全局参数

如果参数名不以 / 开头，如果NodeHandle 对象创建时如果是调用的默认的无参构造，那么该参数是相对参数

如果NodeHandle 对象创建时是使用：ros::NodeHandle nh("~")，那么该参数就是私有参数

```cpp
ros::NodeHandle nh;
nh.setParam("/nh_A",100); //全局,和命名空间以及节点名称无关

nh.setParam("nh_B",100); //相对,参考命名空间

ros::NodeHandle nh_private("~");
nh_private.setParam("nh_C",100);//私有,参考命名空间与节点名称
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```powershell
/nh_A
/xxx/nh_B
/xxx/yyy/nh_C
```

---

# 4.6 ROS分布式通信

ROS是一个分布式计算环境。一个运行中的ROS系统可以包含分布在多台计算机上多个节点。根据系统的配置方式，任何节点可能随时需要与任何其他节点进行通信。

因此，ROS对网络配置有某些要求：

- 所有端口上的所有机器之间必须有完整的双向连接。
- 每台计算机必须通过所有其他计算机都可以解析的名称来公告自己。

1. **设备准备**

   先要保证不同计算机处于同一网络中，最好分别设置固定IP，如果为虚拟机，需要将网络适配器改为桥接模式；

2. **配置文件修改**

   分别修改不同计算机的 /etc/hosts 文件，在该文件中加入对方的IP地址和计算机名:

   主机端:`从机的IP    从机计算机名`

   从机端:`主机的IP    主机计算机名`

   设置完毕，可以通过 ping 命令测试网络通信是否正常。

   > 用shell查看IP地址: ifconfig
   >
   > 用shell查看计算机名称: hostname

3. **配置主机IP**

   配置主机的 IP 地址

   ~/.bashrc 追加

   ```bash
   export ROS_MASTER_URI=http://主机IP:11311
   export ROS_HOSTNAME=主机IP
   ```

   配置从机的 IP 地址，从机可以有多台，每台都做如下设置:

   ~/.bashrc 追加

   ```bash
   export ROS_MASTER_URI=http://主机IP:11311
   export ROS_HOSTNAME=从机IP
   ```

4. **测试**

   1.主机启动 roscore(必须)

   2.主机启动订阅节点，从机启动发布节点，测试通信是否正常

   3.反向测试，主机启动发布节点，从机启动订阅节点，测试通信是否正常

   











