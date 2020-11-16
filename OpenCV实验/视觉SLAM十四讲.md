2020-10-27

# [视觉SLAM十四讲 高翔](https://www.bilibili.com/video/BV16t411g7FR?from=search&seid=13243194313064828696)

[toc]

内容：

* 第一部分：数学基础
  * 概述 Linux
  * 三维空间刚体运动表述 Eigen
  * 李群与李代数 Sophus
  * 相机模型与图像 OpenCV
  * 非线性优化 Ceres，g2o
* 第二部分：视觉SLAM
  * 视觉里程计 OpenCV
  * 后端优化 Ceres，g2o，gtsam
  * 回环检测 DBoW3
  * 地图构建 PCL，Octomap
* 其他
  * 历史回顾和未来

# 第一讲 引言

预备知识：

* 数学：高等数学、线性代数（矩阵论）、概率论
* 编程：C++、linux，了解语法和基本命令即可
* 不提供windows环境下的方案

# 第二讲 SLAM系统概述

slam(simultaneous localization and mapping 同步定位和建图)

小萝卜移动机器人，自主移动能力，需要SLAM地图构建，相机SLAM，定位和建图，内外兼修，误差全局优化

两类常用传感器，外部安装需要贴，GPS失效，携带于机器人本体上的传感器受限制小

相机不一定能取代激光，相机便宜，但数据量大，需光线充足，不能遮挡，环境需要有纹理（不能一片白/黑），局部遮挡待研究，

相机：单目只有一个图像，双目可以有深度，RGB-D通过物理方法测深度

相机本质：二维投影形式记录了三维世界的信息，丢掉了深度

（单目）相机运动起来时，近快远近

（双目）视差

（深度）结构光ToF，主动测量，功耗大，室外太阳光影响大

视觉里程记：相邻图像估计相机运动，基本形式：通过两张图像计算，运动和结构

**后端优化**：从带有噪声的数据中优化轨迹和地图状态估计问题

**回环检测**：

**建图**：

## 2.3 SLAM问题的数学描述

* 离散时间：t，只关心相机采集时刻的位置和地图
* 机器人自身的位置：$x_1, x_2, ...., x_K $ ，构成轨迹
* 地图由许多个路标组成的：$y_1, ... y_N $，而每个时刻，传感器会测量到一部分路标点，得到它们的观测数据

**运动**：从 k-1 时刻到 k 时刻，小萝卜的位置 x 是如何变化的。
$$
x_k = f(x_{k-1}, u_k, w_k), \quad k =1,...,K
$$

 * u_k 是运动传感器的读数或者输入， ωk 为该过程中加入的噪声

**观测**：传感器在位置$x_k$处，探测到了路标$y_j$，产生了一个观测数据 $z_{k,j}$
$$
z_{k,j}=h(y_j, x_K, v_{k,j}), \quad (k,j) \in 某时刻观察到的路标
$$

* v_k,j 是这次观测噪声



**SLAM问题**：当知道运动测量的读数 u ，以及传感器的读数 z 时，如何求解定位问题(估计 x ) 和建图问题(估计 y )?  其实是一个**状态估计问题**：如何通过带有噪声的测量数据，估计内部的、隐藏着的状态变量？

状态估计问题的求解、与两个方程的具体形式，以及噪声服从哪种分布有关。按照

## 2.4 实践：编程基础

### 2.4.1 安装Linux 操作系统

### 2.4.2  Hello SLAM

```shell
vim slambook2/ch2/helloSLAM.cpp
```

```c++
# include <iostream>
using namespace std;

int main(int argc, char **argv)
{
    cout << "Hello SLAM!" << endl;
    return 0;
}
```

编译（用g++编译器）

```shell
sudo apt-get install g++ # 安装g++
g++ helloSLAM.cpp
```

运行程序：

```shell
./a.out
```

### 2.4.3 使用cmake

理论上，任意一个C++程序都可以用 g++ 来编译。但当程序规模越来越大时，一个工程可能有许多个文件夹和源文件，这时输入的编译命令（g++）会越来越长。对于C++项目，使用一些工程管理工具会更加高效。

在一个cmake 工程中，用 cmake 命令生成一个 makefile 文件，然后，用 make 命令根据这个makefile 文件的内容编译整个工程。

先新建一个CMakeLists.txt 文件

```shell
vim slambook2/ch2/CMakeLists.txt
```

编辑内容：

```cmake
# 声明要求的cmake 最低版本
cmake_minimum_required( VERSION 2.8)

# 声明一个cmake 工程
project (HelloSLAM)

# 添加一个可执行程序
# 语法：add_executable (程序名 源代码文件)
add_executable( helloSLAM helloSLAM.cpp)
```

CMakeLists.txt 文件用于告诉 cmake 要对这个目录下的文件做什么事情。

在当前目录下( slambook2/ch2/) ，调用cmake 对该工程进行cmake 编译:

```shell
cmake .
```

cmake 会输出一些编译信息，然后在当前目录下生成一些中间文件，其中最重要的就是MakeFile 。由于MakeFile 是自动生成的，我们不必修改它。现在，用make 命令对工程进行编译（实际调用了g++）:

```shell
make
```

执行生成的可执行程序：

```shell
./helloSLAM
```

不要中间文件，就把中间文件都放在一个中间目录中，编译成功后，删除中间目录即可。所以，更常见的编译cmake 工程的做法如下:

```shell
mkdir build	# 新建中间文件夹build
cd build	# 进入build
cmak ..		# 对上一层文件夹（代码文件夹）进行编译
make
```

### 2.4.4 使用库

在一个C++ 工程中，并不是所有代码都会编译成可执行文件。只有带有main 函数的文件才会生成可执行程序。而另一些代码，我们只想把它们打包成一个东西，供其他程序调用。这个东西叫作库(Library )。

用 cmake 生成库，并且使用库中的函数：

编辑如下 libHelloSLAM.cpp 文件：

```shell
vim slambook2/ch2/libHelloSLAM.cpp
```

```c++
// 这是一个库文件
# include <iostream>
using namespace std;

void printHello()
{
    cout << "Hello SLAM" << endl;
}
```

这个库提供了一个 printHello 函数，调用此函数将输出一条信息。但是它没有main 函数，这意味着这个库中没有可执行文件。我们在CMakeLists.txt 里加上如下内容:

```cmake
add_library( hello libHelloSLAM.cpp)
```

这条命令告诉 cmake ，我们想把这个文件编译成一个叫作"hello" 的库。然后，和上面一样，使用 cmake 编译整个工程：

```shell
cd build
cmake ..
make
```

这时，在build 文件夹中就会生成一个 libhello.a 文件，这就是我们得到的库。在Linux 中，库文件分成静态库和共享库两种。静态库以.a 作为后缀名，共享库以 .so 结尾。所有库都是一些函数打包后的集合，差别在于**静态库每次被调用都会生成一个副本，而共享库则只有一个副本**，更省空间。如果想生成共享库而不是静态库，只需使用以下语句即可。

```shell
add_library( hello_shared SHARED libHelloSLAM.cpp)
```

此时得到的文件就是libhello_shared.so

库文件是一个压缩包，里面有编译好的二进制函数。如果仅有.a 或 .so 库文件，那么我们并不知道里面的函数到底是什么，调用的形式又是什么样的。为了让别人(或者自己)使用这个库，我们需要提供一个头文件，说明这些库里都有些什么。因此，对于库的使用者，只要拿到了头文件和库文件，就可以调用这个库。下面编写libhel10 的头文件。

```shell
vim slambook2/ch2/libHelloSLAM.h
```

```c++
# ifndef LIBHELLOSLAM_H_
# define LIBHELLOSLAM_H_
// 上面的宏定义是为了防止重复引用这个头文件而引起的重定义错误

// 打印一句 Hello 的函数
void printHello();

# endif
```

这样，根据这个文件和我们刚才编译得到的库文件，就可以使用printHello 函数了。最后，我们写一个可执行程序来调用这个简单的函数:

```shell
vim slambook2/ch2/useHello.cpp
```

```cpp
# include "libHelloSLAM.h"

// 使用libHelloSLAM.h 中的printHello() 函数
int main(int argc, char **argv)
{
    printHello();
    return 0;
}
```

然后，在CMakeLists.txt 中添加一个可执行程序的生成命令，链接到刚才使用的库上:

```cmake
add_executable( useHello useHello.cpp)
target_link_libraries( useHello hello_shared)
```

通过这两行语句， useHello 程序就能顺利使用hello_shared 库中的代码了。这个小例子演示了如何生成并调用一个库。请注意，对于他人提供的库，我们也可用同样的方式对它们进行调用，整合到自己的程序中。

### 2.4.5 使用IDE



# 第三讲 三维空间刚体运动

## 3.1 旋转矩阵

### 3.1.1 点、向量和坐标系

点：空间中的基本元素，没有长度，没有体积；

向量：两个点连起来，可以看成从某点指向另一点的一个箭头，向量固定，而其坐标在不同坐标系下坐标不同。

向量的坐标由坐标系确定，坐标系由3个正交的基定义
$$
\vec{a} = \left[ \begin{array}{ccc} e_1 & e_2 &e_3 \end{array} \right]
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
a_1e_1 + a_2e_2 + a_3e_3
$$
向量$\vec{a}$ 在这组基下的坐标是$(a1,a2,a3)^T$ 

向量的运算由坐标运算来表达，对于 $a, b \in \R^3$(三维实数集)

* **加减法**：$a\pm b = \sum\limits_{i}(a_i \pm b_i)$

* **内积**：$ a\cdot d = a^T b = \sum\limits_{i=1}^3a_ib_i=|\vec{a}||\vec{b}|cos\langle a,b \rangle$

* **外积**:
  $$
  \vec{a}\times \vec{b} = 
  \left| \begin{matrix} 
  	\vec{i} & \vec{j} & \vec{k} \\
  	a_1 & a_2 & a_3 \\
  	b_1 & b_2 & b_3
  \end{matrix} \right| =
  \left| \begin{matrix}
  	a_2b_3 - a_3b_2 \\
  	a_3b_1 - a_1b_3 \\
  	a_1b_2 - a_2b_1
  \end{matrix} \right| =
  
  \left[ \begin{array}
  	\\0 & -a_3 & a_2 \\
  	a_3 & 0 & -a_1 \\
  	-a_2 & a_1 & 0
  \end{array} \right] \vec{b} \triangleq a\hat{} b
  $$
  a^是a的反对称矩阵，aT = -a。
  外积的结果是一个向量，它的方向垂直于这两个向量，大小为$|a||b|sin\langle a,b \rangle$ ，是两个向量张成的四边形的有向面积。

### 3.1.2 坐标系间的欧氏变换

基本问题：坐标系之间是如何变化的？进而：如何计算同一个向量在不同坐标系里的坐标？

相机坐标系坐标 和 世界坐标系坐标之间转换：先得到该点针对机器人坐标系的坐标值，再根据机器人位姿变换到世界坐标系中。这个变换关系用变换矩阵 T 来描述。

在SLAM中：

* 固定的世界坐标系和移动的机器人坐标
* 机器人坐标系随着机器人运动而改变，每个时刻都有新的坐标系

两个坐标系之间的变化（刚体运动）由两个部分组成：

* 原点间的平移（平移是一个向量）
* 三个轴的旋转（旋转是一个矩阵）



设某坐标系（e1, e2, e3）发生了一次**旋转**，变成了（e1', e2', e3'）

对于某个固定的向量a（向量本身不随坐标系旋转），坐标关系:
$$
\left[ \begin{array}{ccc}
	e_1, \ e_2, \ e_3 
\end{array} \right]
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
\left[ \begin{array}{ccc}
	e_1^\prime, \ e_2^\prime,\ e_3^\prime  
\end{array} \right]
\left[ \begin{array}{c} a_1^\prime  \\ a_2^\prime  \\ a_3^\prime  \end{array} \right]
$$
左乘 $\left[ \begin{array}{c} e^T_1\\ e^T_2\\ e^T_3 \end{array} \right]$ , 得：左边的系数就变成了单位矩阵
$$
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
\left[ \begin{array}{ccc}
	e^T_1e_1^\prime & e^T_1e_2^\prime & e^T_1e_3^\prime \\
	e^T_2e_1^\prime & e^T_2e_2^\prime & e^T_2e_3^\prime \\
	e^T_3e_1^\prime & e^T_3e_2^\prime & e^T_3e_3^\prime
\end{array}\right] 
\left[ \begin{array}{c} a_1^\prime \\ a_2^\prime \\ a_3^\prime \end{array} \right] \triangleq R\vec{a^\prime}
$$
R 是由两组基之间的内积组成，刻画了前后同一个向量的坐标变换关系。只要旋转是一样的，这个矩阵就是一样的。可以说，矩阵R 描述了旋转本身。因此，称为旋转矩阵( Rotation Matrix )。同时，该矩阵各分量是两个坐标系基的内积，由于基向量的长度为1 ，所以实际上是各基向量夹角的余弦值。所以这个矩阵也叫方向余弦矩阵( Direction Cosin巳Matrix)。我们后文统一称它为旋转矩阵。

**R 是一个正交矩阵（逆=转置），R的行列式为+1**，称为**旋转矩阵**，则旋转矩阵的**集合**就是**特殊正交群**：$SO(n)= \{ R\in \R^{n\times n}|RR^T=I,det(R)=1 \}$ ，n维空间中的旋转，n=3就是三维空间中的旋转。

旋转矩阵描述了两个坐标的欧氏变换关系（向量没变），比如

* $a_1 = R_{12} a_2$
* 反之：$a_2 = R_{21} a_1$
* 于是：$R_{21}=R_{12}^{-1}=R_{12}^T$
* 进一步，三个坐标系亦有：$a_3=R_{32}a_2=R_{32}R_{21} a1=R_{31}a_1 $

R12 是指 “把坐标系2 的向量变换到坐标系1” 中。由于向量乘在这个矩阵的右边，它的下标是从右读到左的。这也是本书的习惯写法。坐标变换很容易摘混，特别是存在多个坐标系的情况下。同理，如果我们要表达"从1 到2 的旋转矩阵"时，就写成R21 。

关于平移t12 ， 它实际对应的是坐标系1 原点指向坐标系2 原点的向量，在坐标系1 下取的坐标，所以笔者建议读者把它记作"从1 到2 的向量"。但是反过来的 t21 ， 即从2 指向1 的向量在坐标系2 下的坐标，却并不等于-t 12 ， 而是和两个系的旋转还有关系①。

所以，当初学者问 "我的坐标在哪里" 这样的问题时，我们需要清楚地说明这句话的含义。这里"我的坐标"实际上指从世界坐标系指向自己坐标系原点的向量，在世界坐标系下取到的坐标。对应到数学符号上，应该是 twc 的取值。同理，它也不是-tcw

### 3.1.3 变换矩阵与齐次坐标

旋转加上平移：$a'=Ra+t$, 两个坐标系的刚体运动可以由R，t 完全描述。

但是如果发生了两次变换：$b=R_1a+t, \ c=R_2b+t_2$

这时：$c = R_2(R_1a+t_1)+t_2$，叠加起来过于复杂

把一次平移和一次旋转合成一个矩阵 T：
$$
\left[ \begin{array}{c} a^\prime \\ 1 \end{array} \right]=
\left[ \begin{array}{cc} R & t \\ 0^T & 1 \end{array} \right]
\left[ \begin{array}{c} a \\ 1 \end{array} \right] \triangleq
T \left[ \begin{array}{c} a \\ 1 \end{array} \right]
$$
记 $\tilde{a}=\left[ \begin{array}{c} a \\1 \end{array} \right]$ ，这种在三维向量的末尾添加 1（四维向量）表达三维向量的做法称为**齐次坐标**，引入齐次坐标后，旋转和平移可以放入同一个矩阵，称为变换矩阵 T 。那么多次变换就可以写成：
$$
\tilde{b}=T_1 \tilde{a},\tilde{c}=T_2 \tilde{b} \Rightarrow \tilde{c}=T_2 T_1 \tilde{a}
$$
**特殊欧氏群**(special Euclidean Group)：
$$
SE(3)=\left\{ T= \left[ \begin{array}{cc} R & t \\ 0^T & 1 \end{array} \right] \in R^{4\times 4}|R \in SO(3),t \in R^3 \right\}
$$
反向变换：
$$
T^{-1}=\left[ \begin{array}{cc} R^T & -R^Tt\\ 0^T & 1 \end{array} \right]
$$
在SLAM中，通常定义世界坐标系$T_W$ 与 机器人坐标系$T_R$ ，一个点的世界坐标为$p_R$，那么满足关系：
$$
p_R = T_{RW}p_W
$$
在实际编程中，可使用$T_{RW}$ 或 $T_{WR}$ 来描述机器人的位姿。

相机轨迹画的是相机坐标系中心在世界坐标系的坐标

## 3.2 EIGEN

C++线性代数库，运行效率高

ubuntu 安装eigen 

```shell
sudo apt-get install libeigen3-dev
```

只有头文件库：/usr/include/eigen3 没有库文件，不需要链接库，只要把头文件加进来就可以了

eigen 和matlab像，所以东西都是矩阵类型定义的 ，基本数据类型是 Eigen::Matrix

CMakeLists.txt：

```cmake
cmake_minimum_required( VERSION 2.8 )
project( useEigen )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_Cxx_FLAGS ".03" )
# 添加Eigen 头文件
include_directories( "/usr/include/eigen3" )

add_executable( eigenMatrix eigenMatrix.cpp)
```

eigenMatrix.cpp

```c++
# include <iostrems>
using namespace std:
# include <ctime>

// Eigen 部分
# include <Eigen/Core>	//提供核心矩阵运算
# include <Eigen/Dense>	//稠密矩阵的代数运算（逆、特征值等）
# define MATRIX_SIZE 50

/*****************
* 本程序演示了Eigen基本类型的使用
*****************/

int main( int argc, char** argv )
{
    // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类，它的三个参数为：数据类型，行，列
    // 声明一个2*3 的float 矩阵
    Eigen::Matrix<float, 2, 3>matrix_23
}
```

## 3.3 旋转向量、欧拉角

除了旋转矩阵之外的旋转表示：旋转向量，欧拉角，四元数

旋转可以表示为：绕某一轴旋转多少角度，这个轴称为角轴，或旋转向量，长度为1，
$$
w = \Theta n
$$
角轴与旋转矩阵不同：

* 旋转矩阵：9个量，有正交性约束和行列式值约束
* 角轴：三个量，没有约束

注意它们只是表达方式的不同，但表达的东西可以是同一个，角轴也就是第四章要介绍的李代数

转换关系：

 * 轴角转换到旋转矩阵：**罗德里格斯公式**
   $$
   R = cos\theta I + (1-cos\theta)n n^T +sin\theta n \verb|^|
   $$

* 旋转矩阵转轴角

  * 角度：$\theta = arccos(\frac{tr(R)-1}{2})$
  * 轴：$Rn = n$

**欧拉角**：（对人类直观）

* 将旋转分解为三次不同轴上的转动，
* 例如：按Z-Y-X 顺序转动
* 轴可以是定轴或动轴，顺序亦可不同，因此存在许多种定义方式不同的欧拉角
* 常见的有yaw-pitch-roll（偏航-俯仰-滚转）角等
  * 绕物体的Z轴旋转，得到偏航角yaw
  * 绕旋转之后的Y轴旋转，得到俯仰角pitch
  * 绕旋转之后的X轴旋转，得到滚转角roll

万向锁（Gimbal Lock）问题：

* ZYX顺序中，若Pitch为正负90度，则第三次旋转和第一次绕同一个轴，使得系统丢失了一个自由度——存在奇异性问题（所以不常用其表达姿态，往往用于人机交互中）

## 3.4 四元数

紧凑但没奇异性的表述方式。

* 四元数是一种扩展的复数

* 回忆：（单位圆上的）复数可以表达**二维平面**的旋转：x乘上一个i，逆时针旋转90度

* 四元数有三个虚部，可以表达**三维空间**中的旋转
  $$
  \vec{q} = q_0+q_1i+q_2j+q_3k, \\
  $$
  也可写成有实部和虚部组成的向量：
  $$
  \vec{q} = [\vec{s}, \vec{v}], 
  \vec{s} = q_0 \in \R, \vec{v}=[q_1,q_2,q_3]^T\in \R^3
  $$

* 虚部之间的关系：
  $$
  \left\{
  \begin{array}{c}
  i^2=j^2=k^2=-1	\\
  ij=k, \ ji=-k	\\
  jk=i, \ kj=-i	\\
  ki=j, \ ik=-j
  \end{array}
  \right.
  $$
  自己和自己的运算像复数，自己和别人的运算像三维空间里的叉乘

* 和复数一样，单位四元数可以表达三维空间的一次旋转

* 四元数的一些运算和性质：

  * 加减法
    $$
    q_a\pm q_b = [s_a \pm s_b, v_a \pm v_b]
    $$

  * 乘法
    $$
    \begin{align}
    q_aq_b & =s_as_b-x_ax_b-y_ay_b-z_az_b\\
    		&+(s_a x_b+x_a s_b+y_a z_b-z_a y_b)i \\
    		&+(s_a y_b-x_a z_b+y_a s_b+z_a x_b)j\\
    		&+(s_a z_b+x_a y_b-y_a x_a+z_a s_b)k
    \end{align}
    $$

  * 乘方
    $$
    q_aq_b = [s_as_b - v_a^Tv_b,\ s_av_b - s_bv_a + v_a\times v_b]
    $$

  * 共轭 

  * 模长

  * 逆

  * 数乘

  * 点乘

四元数表达三维空间旋转

* 四元数和角轴的关系

  * 角轴到四元数
    $$
    q = [cos\frac{\theta}{2},\ n_xsin\frac{\theta}{2},\ n_ysin\frac{\theta}{2},\ n_zsin\frac{\theta}{2} ]^T
    $$

  * 四元数到角轴
    $$
    \left\{
    \begin{array}{c}
    \theta & = & 2 arccosq_0\\
    [n_z,n_y, n_z]^T & = & [q_1,q_2,q_3]^T/sin\frac{\theta}{2}
    \end{array}
    \right.
    $$

  * 类似可知四元数亦可可转换为旋转矩阵、欧拉角

用四元数旋转一个空间点：

* 设点 p 经过一次以 q 表示的旋转后，得到了 p' ，它们关系如何表示？

  1. 设p 的坐标(x,y,z) 用四元数表示（**虚四元数**）：$\vec{p}=[0,x,y,z] = [0,\vec{v}]$

  2. 旋转之后的关系为：
     $$
     p^\prime = qpq^{-1}
     $$
     可以验证这也是四元数

* 四元数相比于角轴、欧拉角的优势：**紧凑、无奇异性**

## 3.6 实践：EIGEN 几何模块

# 第四讲 李群与李代数

本讲目标：

> * 理解李群与李代数的概念，掌握SO(3)，SE(3)与对应李代数的表示方式。
> * 理解李代数上的求导的方式和意义。
> * 使用Sophus对李代数进行运算。

在SLAM 中，除了表示，我们还要对它们进行估计和优化。因为在SLAM 中位姿是未知的，而我们需要解决形如"**什么样的相机位姿最符合当前观测数据"**这样的问题。一种典型的方式是把它构建成一个优化问题，求解最优的
R， t ， 使得误差最小化。
如前所言，旋转矩阵自身是带有约束的(正交且行列式为1，非线性约束 )。它们作为优化变量时，会引入额外的约束，使优化变得困难。通过李群一李代数间的转换关系，我们希望把位姿估计变成无约束的优化问题，简化求解方式

## 4.1 李群李代数基础

三维旋转矩阵构成了**特殊正交群SO(3)**
$$
SO(3)= \{ R\in R^{3\times 3}|RR^T=I,det(R)=1 \}
$$


三维变换矩阵构成了**特殊欧氏群SE(3)**
$$
SE(3)=\left\{ T= \left[ \begin{array}{cc} R & t \\ 0^T & 1 \end{array} \right] \in R^{4\times 4}|R \in SO(3),t \in R^3 \right\}
$$
**群**(Group)：是一种**集合**加上一种**运算**的**代数结构**。

记集合为A，运算记为 · ，那么当运算满足以下性质时，称（A, · ）成群：

1. 封闭性：$\forall a_1,a_2 \in A,\quad a_1\cdot a_2 \in A$

2. 结合律：$\forall a_1,a_2,a_3 \in A, \quad (a_1·a_2)·a_3 = a_1·（a_2·a_3）$

3. 幺元：$\exists a_0 \in A,\quad s.t. \quad \forall a \in A, \quad a_0·a=a·a_0=a$

4. 逆：$\forall a \in A, \quad \exists a^{-1} \in A, \quad s.t. \quad a·a^{-1} = a_0$

   封结幺逆（“凤姐咬你”）

可以验证：

* **旋转矩阵集合**和**矩阵乘法**构成群。满足封闭性和结合律；幺元是I；逆：往左转的旋转和往右转的旋转相乘等于I（是幺元）
* 同样**变换矩阵**和**矩阵乘法**也构成群。
* 因此称它们为**旋转矩阵群**(SO(3))和**变换矩阵群**(SE(3))。

其他常见的群：

* 一般线性群GL(n)	指 n x n 的可逆矩阵，它们对矩阵乘法成群
* 特殊正交群SO(n)    也就是所谓的旋转矩阵群，其中SO(2) 和 SO(3) 最为常见。
* 特殊欧氏群SE(n)    也就是前面提到的n维欧氏变换，如SE(2) 和 SE(3)。

群结构保证了在**群上的运算**具有良好的性质。

李群(Lie Group):

* 是**连续**（光滑）的群
* 既是群也是流形
* 直观上看，因为一个刚体能够连续地在空间中运动，故SO(3)和SE(3)都是李群
* 但是，SO(3) 和 SE(3) 只有一个运算：乘法（如果研究一个集合和两个运算需要环论和域论)，没有加法，所以难以进行**取极限、求导**等操作。

李代数：

* 是与李群**一一对应**的一种结构，位于向量空间（旋转矩阵群对应一种李代数）
* 通常记作**小写的so(3) 和 se(3)**。书中以哥特体突出显示
* 事实上是李群单位元处的正切空间。

李代数的引出：

* 任意旋转矩阵R（是正交矩阵），满足：
  $$
  RR^T = I
  $$

* 李群是连续的，所以还考虑R随时间的变化，有：
  $$
  R(t)R(t)^T = I
  $$

* 两侧对时间求导：
  $$
  \dot{R(t)} R(t)^T + R(t)\dot{R(t)^T} = 0
  $$

* 整理得：
  $$
  \dot{R(t)} R(t)^T = -(R(t)\dot{R(t)^T})^T
  $$

* 可以看出这是一个反对称矩阵(转置之后多负号)，记
  $$
  \dot{R(t)} R(t)^T = \phi(t)\hat{}
  $$
  反对称符号：
  $$
  \begin{align*}
  & a^{\wedge} = A = 
  \left[
  \begin{array}{ccc} 0 & -a3 & a_2 \\
  a_3 & 0 & -a_1 \\
  -a_2 & a_1 & 0\\
  \end{array}
  \right],\\
  & A^{\vee}= a
  \end{align*}
  $$
  
* 两侧右乘R(t)，得
  $$
  \dot{R(t)} = \phi(t)^{\wedge} R(t)
  $$

* 可以看到，对R求导后，左侧多出一个 $\phi(t)\hat{}$

然后求这个微分方程：

* 考虑简单情况：$t_0 = 0, R(0)=I$, 泰勒展开：
  $$
  \begin{align}
  R(t) & \approx R(t_0)+\dot{R(t_0)}(t-t_0)\\
  & = I+\phi(t_0)^{\wedge}(t)
  \end{align}
  $$

* 可见$\phi$ 反映了一阶导数性质，它位于**正切空间**(tangent space) 上

* 在t0 附近，假设$\phi$ 不变，则上面的微分方程写成：
  $$
  \dot{R(t)}=\phi(t_0)\hat{}R(t) = \phi_0\hat{}R(t)
  $$
  类似常系数的微分方程，每求一次导，前面多乘一个$\phi_0\hat{}$ , 这个函数就类似指数函数

* 已知初始情况：$R(0)=I$, 解之，得：
  $$
  R(t)=exp(\phi_0\hat{}t)
  $$

* 该式说明，对t0附近的任意t，都可以找到一个R和一个$\phi$ 的对应关系

  * 该关系称为指数映射(Exponential Map)
  * 这里的$\phi$ 称为SO(3) 对应的李代数：so(3)，是一个三维空间向量

**李代数**(Lie Algebra):

* 每个李群都有与之对应的李代数。李代数描述了李群单位元数的正切空间性质

* **李代数**由**一个集合V，一个数域F 和一个二元运算[ , ] 组成**。如果它们满足以下几条性质，称 (V, F,[ , ]) 为一个李代数，记作

  1. 封闭性  $\forall X,Y \in V, [X,Y] \in V$

  2. 双线性  $\forall X,Y,Z \in V, a,b \in F$, 有：

     $[aX+bY, Z]=a[X,Z]+b[Y,Z],\\ [Z, aX+bY]=a[Z,X]+b[Z,Y]$

  3. 自反性  $\forall X \in V, [X,X]=0$

  4. 雅可比等价  $\forall X,Y,Z \in [X,[Y,Z]]+[Z,[Y,X]],[Y,[Z,X]]=0$

* 其中二元运算[ , ] 被称为**李括号**(Lie Bracket).

  直观上说，李括号表达了两个元素的差异。

* 例子：三维空间向量+叉积运算 构成李代数

* **旋转群李代数** so(3): 
  $$
  so(3)=\{\phi \in \R^3, \Phi=\phi^{\wedge} \in \R^{3\times 3} \}
  $$
  

  其中：$\Phi = \phi\hat{}=\left[\begin{array}{ccc} 0 & -\phi_3 & \phi_2 \\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{array}\right] \in \R^{3 \times 3}$ 

  **李括号**：$[\phi_1, \phi_2] = (\Phi_1\Phi_2 - \Phi_2\Phi_1)^{\vee} $

  

* **变换群李代数** se(3): 有6个自由度
  $$
  se(3)=\left\{
  \xi = \left[ \begin{array}{cc}\rho \\ \phi\end{array} \right]\in \R^6, \rho\in\R^3,\phi\in so(3), \xi\hat{}=\left[ \begin{array}{cc}\phi\hat{} & \rho \\ 0^T & 0 \end{array} \right] \in \R^{4\times4}
  \right\}
  $$
  se(3) 由三个平移分量 和 三个旋转分量组成（\rho 在前还是\phi 在前都一样）
  	旋转$\phi$ 与 so(3) 相同
  	平移$\rho$ 是普通的三维向量，但不是SE3上的平移分量！

  上尖尖^ 不再是反对称矩阵，但仍保留记法。

  李括号：$[ \xi_1,\xi_2] = (\xi_1^{\wedge}\xi_2^{\wedge} - \xi_2^{\wedge} \xi_1^{\wedge})^{\vee}$

* 把李代数理解成向量形式或矩阵形式都是可以的。向量形式更加自然一些。

## 4.2 指数映射和对数映射

指数映射反应了从李代数到李群的对应关系：
$$
R = exp(\phi^{\wedge})
$$
但是 $\phi\hat{}$是一个矩阵，对于矩阵，如何定义求指数运算？ —— Taylor 展开：指数展开等于级数求和
$$
exp(\phi\hat{})=\Sigma_{n=0}^\infin \frac{1}{n!}(\phi\hat{})^n
$$


由于$\phi$ 是向量，定义其和模长：

* 模长乘方向向量：$\phi=\theta a$

* 关于$a$，可以验证以下性质：
  $$
  a\hat{} a\hat{} = a a^T - I,\\
  a\hat{} a\hat{}a\hat{} = -a\hat{}
  $$
  这为化简Taylor展开式中的高阶项提供了有效方法

* 最后得到一个似曾相识的结果：罗德里格斯公式
  $$
  exp(\theta a\hat{}) = cos\theta I +(1-cos\theta)a a^T + sin\theta a^T
  $$
  李代数到李群的指数映射也是罗德里格斯公式

* 说明了李代数**so(3)** 的物理意义就是**旋转向量**

* 反之，给定旋转矩阵时，对数映射亦能求李代数：
  $$
  \phi = ln(R)\check{} = (\sum_{n=0}^\infin \frac{(-1)^n}{n+1} (R-I)^{n+1})\check{}
  $$

* 但实际中没必要这样求，在旋转向量小节已经介绍了矩阵到向量的转换关系：
  $$
  模长：\theta = arccos(\frac{tr(R)-1}{2}) \\
  方向：Rn = n（绕旋转轴不变）
  $$

* 至此，说明了SO(3) 与 so(3) 的对应关系

**se(3) 到 SE(3) 的指数映射** ：
$$
\begin{align}
exp(\xi\hat{}) &= \left[ \begin{array}{cc}
\sum\limits_{n=0}^\infin \frac{1}{n!}(\phi\hat{})^n & \sum\limits_{n=0}^\infin \frac{1}{(n+1)!}(\phi\hat{})^n\rho \\
0^T	& 1 \end{array} \right]\\

& \triangleq \left[ \begin{array}{cc}
R & J\rho \\
0^T & 1
\end{array} \right] \\

& = T
\end{align}
$$
​	左上角表示**李代数的平移部分**到矩阵的平移部分相差一个**线性变换**，由雅可比矩阵 J 给出
$$
J = \frac{sin\theta}{\theta}I + (1- \frac{sin\theta}{\theta})a a^T + \frac{1-cos\theta}{\theta}a\hat{}
$$
![image-20201029153253948](C:\Users\MY\Documents\GitHub\cloudimg\旋转矩阵和变换矩阵的李群与李代数.png)

## 4.3 李代数求导与扰动模型

因为李群没有加法，不能定义导数 $R_1 + R_2 \notin SO(3)$

直观的想法：

	* 能否利用李代数上的加法，定义李群元素的导数？
	* 使用指数映射和对数映射完成变换关系。

基本问题：当在李代数中做加法时，是否等价于在李群上做乘法？
$$
exp(\phi_1\hat{}) exp(\phi_2\hat{}) = exp((\phi_1+\phi_2)\hat{}) ?
$$
在使用标量的情况下，该式明显成立。但这里的$\phi\hat{}$ 为 矩阵 ！,不成立

完整形式由BCH (Baker-Campbell-Hausdorff) 公式给出：完整形式见wikipedia

部分展开式：（方括号为李括号）
$$
ln(exp(A)exp(B)) = A + B + \frac{1}{2}[A,B]+\frac{1}{12}[A,[A,B]] - \frac{1}{12}[B,[A,B]]+...
$$
最基本的近似可以看成A+B，但后面还有很多李括号。

比如A特别小，平方以上的项就不要了，只取一次项

当其中一个量为小量时，忽略其高阶项，BCH具有线性近似形式：
$$
ln(exp(\phi_1\hat{})exp(\phi_2\hat{}))\check{} \approx \left\{
\begin{array}{cc}
J_l(\phi_2)^{-1}\phi_1 + \phi_2 & \text{if}\ \ \phi_1  \ \text{is small} \\
J_r(\phi_1)^{-1}\phi_2 + \phi_1 & \text{if}\ \ \phi_2  \ \text{is small}
\end{array}
\right.
$$
这里的 左乘雅可比:
$$
\begin{array}{cc}
J_l &=& J = \frac{sin\theta}{\theta}I+(1-\frac{sin\theta}{\theta})aa^T + \frac{1-cos\theta}{\theta}a\hat{} \\
J_l^{-1} &=& \frac{\theta}{2}cot\frac{\theta}{2}I+(1-\frac{\theta}{2}cot\frac{\theta}{2})aa^T - \frac{\theta}{2}a\hat{}
\end{array}
$$
右乘雅可比：
$$
J_r(\phi) = J_l(-\phi)
$$
直观写法（以左乘为例）：
$$
exp(\Delta\phi\hat{})exp(\phi\hat{}) = exp((\phi + J_l^{-1}(\phi)\Delta\phi)\hat{})
$$

* 在李群上左乘小量时，李代数上的加法相差左雅可比的逆

反之：
$$
exp((\phi+\Delta\phi)\hat{}) = exp((J_l\Delta\phi)\hat{})exp(\phi\hat{})=exp(\phi\hat{})exp((J_r\Delta\phi)\hat{})
$$

* 李代数上进行小量加法时，相当于李群上左（右）乘一个带左（右）雅可比的量

se(3) 上形式更为复杂：
$$
exp(\Delta\xi\hat{})exp(\xi\hat{}) \approx exp((J_l^{-1} \Delta\xi+\xi)\hat{}),\\
exp(\xi\hat{})exp(\Delta\xi\hat{}) \approx exp((J_r^{-1} \Delta\xi+\xi)\hat{})
$$
通过BCH 线性近似，可以定义李代数上的导数：

考虑一个基本问题：旋转后的点关于旋转的导数，不严谨的记为：$\frac{\partial({R_p})}{\partial{R}}$

由于R 没有加法，导数无从定义，存在两种解决办法：

* 对R 对应的李代数加上小量，求相对于小量的变化率（导数模型）；
* 对R左乘或右乘一个小量，求相对于小量的李代数的变化率（扰动模型）

导数模型：

* 按照定义可推得：

$$
\begin{array}{cc}
\frac{\partial(exp(\phi\hat{})p)}{\partial\phi} &=& \lim_{\delta\phi \to 0} 
\frac{exp((\phi+\delta\phi)\hat{})p-exp(\phi\hat{})p}{\delta\phi} \\
& \text{BCH公式近似写开：}\\
& = &\lim_{\delta\phi \to 0} \frac{exp((J_l\delta\phi)\hat{})exp(\phi\hat{})p-exp(\phi\hat{})p}{\delta\phi} \\
& exp泰勒展开，保留一阶项：\\
& \approx&  \lim_{\delta\phi \to 0} \frac{(I+(J_l\delta\phi)\hat{})exp(\phi\hat{})p-exp(\phi\hat{})p}{\delta\phi}\\
& =& \lim_{\delta\phi \to 0} \frac{(J_l\delta\phi)\hat{}exp(\phi\hat{})p}{\delta\phi} \\
& 反对称符号与叉积符号是一样的，\\&说明可以交换位置：\\
& = & \lim_{\delta\phi \to 0} \frac{-(exp(\phi\hat{})p)\hat{}J_l\delta\phi}{\delta\phi} \\
& = & -(Rp)\hat{}J_l
\end{array}
$$

结果中含有左乘雅可比Jl，比较复杂，能否避免？

扰动模型（左乘）：

* 左乘小量，令其李代数为零：
  $$
  \begin{array}{cc}
  \frac{\partial(R_p)}{\partial\varphi} &=& \lim_{\varphi \to 0} \frac{exp(\varphi\hat{})exp(\phi\hat{})p-exp(\phi\hat{})p}{\varphi} \\
  & = & \lim_{\varphi \to 0} \frac{(1+\varphi\hat{})exp(\phi\hat{})p-exp(\phi\hat{})p}{\varphi} \\
  & = & \lim_{\varphi \to 0} \frac{\varphi\hat{}Rp}{\varphi} \\
  &=& \lim_{\varphi \to 0} \frac{-(Rp)\hat{}\varphi}{\varphi} \\
  &=& -(Rp)^\wedge
  
  \end{array}
  $$
  
* 最终结果没有Jl，更为简洁，更实用。

SE(3)上的扰动模型：
$$
\begin{array}{cc}
\frac{}{}
\end{array}
$$
小结：

* 利用BCH 线性近似，可以推导so(3) 和 se(3) 上的导数和扰动模型
* 通常情况下，扰动模型形式比较简洁，所以更常用

## 演示：Sophus库



# 第五讲 相机与图像

之前讲了三维世界中刚体运动的描述：旋转矩阵、旋转向量、欧拉角、四元数等。以及优化：李群与李代数
$$
\left\{
\begin{align}
x_k &= f(x_{k-1},u_k,w_k)	&\text{运动方程}\\
z_{k,j} & = h(y_j,x_k,v_{k,j}) &\text{观测方程}
\end{align}
\right.
$$
观测传感器是相机，所以观测方程是相机模型

## 5.1 相机针孔模型

### 5.1.3 双目相机模型

针孔相机模型描述了单个相机的成像模型。然而，仅根据一个像素，我们无法确定这个空间点的具体位置，深度不知道。

双目相机通过同步采集左右相机的图像，计算图像间视差，以便估计每一个像素的深度。

两个镜头水平放置，两个光圈中心都位于x轴上，两者之间的距离称为双目相机的**基线**。

视差d本身的计算：需要知道左眼图像的某个像素出现在右眼图像的哪个位置（即对应关系）。当我们想计算每个像素的深度时，其计算量与精度都将成为问题，而且只有在图像纹理变化丰富的地方才能计算视差。由于计算量的原因，双目深度估计仍需要使用GPU 或FPGA 来实时计算。这将在第13 讲中介绍。



## 5.2 图像

图像在计算机中以矩阵形式存储

## 5.3 实践：基本图像处理

### 5.3.1 OpenCV的基本使用方法

OpenCV 提供了大量的开源图像算法，是计算机视觉中使用极广的图像处理算法库。

#### 安装OpenCV

在Ubunt 系统下，有从**源代码安装**和**只安装库文件**两种方式可以选择

1. 从源代码安装，是指从OpenCV网站下载所有的OpenCV 源代码，并在机器上编译安装，以便使用。好处是可以选择的版本比较丰富，而且能看到源代码，不过需要花费一些编译时间。
2. 只安装库文件，是指通过Ubuntu 安装由Ubuntu 社区人员已经编译好的库文件，这样就无须重新编译一遍。

从http://opencv.org/downloads.html 下载，选择OpenCV for Linux 版本即可。你会获得一个像opencv-3. 1. 0.zip 这样的压缩包。将它解压到任意目录下，我们发现OpenCV 也是一个cmake 工程。
在编译之前，先来安装OpenCV 的依赖项：

```shell
sudo apt-get install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev libtbb-dev
```

#### 操作OpenCV图像

```c++
# include <iostream>

```





## 5.4 实践：3D视觉

### 5.4.1 双目视觉

们从双目视觉的左右图像出发，计算图像对应的视差图，然后计算各像素在相机坐标系下的坐标，它们将构成点云。在第5 讲代码目录的stereo 文件夹中，我们准备了双目视觉的图像，如图5-9 所示。下面的代码演示了计算视差图和点云部分:

```shell
vim slambook/ch5/stereoVision/stereoVison.cpp
```

```c++
int main (int argc, char **argv)
{
    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    //基线
    double b = 0.573;
    
    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32*9*9, 1, 63, 10, 100, 32);//神奇的参数
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0/16.0f);
    
    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;
    
    // 如果你的机器慢，把后面的v++ 和 u++ 改成 v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++)
        {
            if (disparity.at<float>(v, u) <= 10.0 || disparity.at<float>(v, u) >= 96.0)
                continue;
            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);	//前三维为xyz，第四维为颜色
            
            // 根据双目模型计算point的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v,u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;
            
            pointcloud.push_back(point);
        }
	cv::imshow("disparity", disparity / 96.0);
    cv::waitKey(0);
    // 画出点云
    showPointCloud(pointcloud);
    return 0;
}
```

### 点云拼接



# 第六讲 非线性优化

本讲目标

* 理解最小二乘法的含义和处理方法
* 理解Gauss-Newton，Levenburg-Marquadt等下降策略
* 学习Ceres库 和 g2o库的基本使用方法：求解最小二乘

上讲回顾：

* 三维世界中刚体的描述：旋转矩阵、旋转向量、欧拉角、四元数等

* 观测模型：相机投影模型

* $$
  \left\{
  \begin{align}
  x_k &= f(x_{k-1},u_k,w_k)	&\text{运动方程}\\
  z_{k,j} & = h(y_j,x_k,v_{k,j}) &\text{观测方程}
  \end{align}
  \right.
  $$

* 

在给定模型和具体观测时（已知运动方程和观测方程具体形式），如何对估计进行优化？

## 6.1 状态估计问题

$$
\left\{
\begin{align}
x_k &= f(x_{k-1},u_k)+w_k	&\text{运动方程}\\
z_{k,j} & = h(y_j,x_k)+v_{k,j} &\text{观测方程}
\end{align}
\right.
$$

从噪声中估计出状态

xk是6自由度的pose用变换矩阵或李代数的指数来表示：$x_k = T_k = exp(\xi_k\hat{})$

$sz_{k,j} = K exp(\xi\hat{})y_j$ ：z是像素，外参乘坐标点乘内参再除距离就得到像素，

$w_k \sim N(0,R_k)是运动噪声，v_{k,j}\sim N(0,Q_{i,j})是观测噪声$, 

根据方程形式，噪声满足的概率分布分类：

* 如果f，h是线性方程（几个量加和，数乘，组成的线性系统），并且噪声服从高斯分布：线性高斯系统（最简单情况，由卡尔曼滤波做出最优误偏估计）
* 如果f，h是非线性函数（由几个量乘在一起），非高斯噪声，需要分情况讨论

历史上很长时间用滤波器求解状态估计，假设系统具有马尔可夫性，系统的下一时刻的状态仅依赖上一时刻的状态，可以说，只维护当前的状态，当有新输入进入系统中时，更新当前状态的估计。

但是近年非线性优化已成为主流，我们首先来分析一下如何从概率角度看待此问题。

状态变量（把所有代求量放到一块）
$$
x = \{x_1,....,x_N, y_1, ... y_M\} \quad 所有时刻的位姿和路标
$$
状态估计等同于求解条件分布，当我知道z 和 u 时求在此条件x的概率分布，就可确定状态。这里z，u也是统称

考虑更简单的情况：只有观测方程时（没有运动方程），只考虑z对x的影响，不考虑u，类似于sfm。其实slam某种程度与其很像，不过slam图像有时间先后

贝叶斯法则：
$$
P(x|z) = \frac{P(z|x)P(x)}{P(z)} \propto P(z|x)P(x)
$$
P(z|x) 是似然，P(x)是先验，因为P(x|z) 条件分布很难求解(模型难选定，参数难估计)，但可以求：

* **最大后验估计**（maximize a Posterior, MAP) 使得P(x|z)这个**数**最大

  $x*_{MAP} = arg max P(x|z) = arg max P(z|x)P(x)$

* **最大似然估计**（maximize Likelihood Estimation, MLE) ，不知道先验P(x)，只估计似然

  $x*_{MLE}=arg max P(z|x)$

  “在哪种状态下，最容易产生当前的观测”

从最大似然到最小二乘：

* 例：某一次观测 $z_{k,j}=h(y_j,x_k)+v_{k,j}$，由于噪声是高斯的：$v_k \sim N(0,Q_{k,j})$，于是$P(z_{j,k}|x_k,y_j)=N(h(y_j,x_k),Q_{k,j})$ .现在要求x,y的最大似然估计怎么做？

一般的高斯分布：
$$
P(x) = \frac{1}{\sqrt{(2\pi)^N det(\Sigma)}}exp(-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu))
$$
负对数形式：(简化形式，只有第二项与x有关)
$$
-ln(P(x))=\frac{1}{2}ln((2\pi)^Ndet(\sum))+\frac{1}{2}(x-\mu)^T\sum^{-1}(x-\mu)
$$
原问题的最大化，相当于**负对数最小化**

因此，关于原问题的最大似然：
$$
P(z_{j,k}|x_k,y_j)=N(h(y_i,x_k),Q_{k,j})
$$
相当于最小化：
$$
x^{*} = arg\ min((z_{k,j}-h(x_k,y_j))^TQ^{-1}_{k,j}(z_{K,j}-h(x_k,y_j)))
$$
上面就是所谓的最小二乘。

对于原问题，每一个运动方程和观测方程，高斯分布假设，对当前估计值xk求误差，运动误差=估计值减上一时刻推出来的值；观测误差=真实zkj-在当前xk的观测yj的观测量：
$$
e_{v,k}=x_k - f(x_{k_1},u_k)  运动误差\\
e_{y,j,k} = z_{k,j} - h(x_k,y_j)  观测误差
$$
最小化误差的二范数：
$$
min\ J(x)=\sum_k e_{v,k}^T R_k^{-1}e_{v,k}+\sum_k \sum_j e_{y,k,j}^T Q_{k,j}^{-1} e_{y,k,j}
$$



# 第七讲 特征点法的视觉里程计

# 第八讲 直接法的视觉里程计

# 第九讲 后端优化



# 第十讲 位姿图

# 第十一讲 回环检测

# 第十二讲 地图构建

# 第十三讲 工程实践

# 第十四讲 开源SLAM方案



