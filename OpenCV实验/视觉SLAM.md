2020-10-27

# [视觉SLAM十四讲 高翔](https://www.bilibili.com/video/BV16t411g7FR?from=search&seid=13243194313064828696)

**内容：**

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

## 第一讲 引言

预备知识：

* 数学：高等数学、线性代数（矩阵论）、概率论
* 编程：C++、linux，了解语法和基本命令即可
* 不提供windows环境下的方案

## 第二讲 初识SLAM

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

**SLAM问题的数学描述**

离散时间：t

机器人的位置：x

都是随机变量，服从概率分布，

小萝卜是从上一个时刻运动到下一个时刻的

运动方程：

路标（三维空间点）：y1.。。yn

传感器在位置xk处，探测到了路标yj

观测方程：$z_{k,j}=h()$

两个基本方程：

​	运动方程：
​	观测方程:

位置是三维的，如何表述

观测是相机中的像素点，如何表述

已知u，z时，如何推断x，y

### slam 实践部分



## 第三讲 三维空间刚体运动

向量的坐标由坐标系确定，坐标系由3个正交的基定义
$$
\vec{a} = \left[ \begin{array}{ccc} e_1 & e_2 &e_3 \end{array} \right]
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
a_1e_1 + a_2e_2 + a_3e_3
$$
向量的坐标是a1,a2,a3

向量的运算由坐标运算来表达

加减法：$a\pm b = \Sigma_{i}(a_i \pm b_i)$

内积：$ a\cdot d = a^T b = \Sigma_{i=1}^3a_ib_i=|\vec{a}||\vec{b}|cos(a,b)$

外积:
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
\end{array} \right] \vec{b} \triangleq a\verb|^| b
$$
a^是反对称矩阵：aT = -a

基本问题：坐标系之间是如何变化的？进而：如何计算同一个向量在不同坐标系里的坐标？

在SLAM中：

* 固定的世界坐标系和移动的机器人坐标
* 机器人坐标系随着机器人运动而改变，每个时刻都有新的坐标系

两个坐标系之间的变化由两个部分组成：

* 原点间的平移（平移是一个向量）
* 三个轴的旋转（旋转是一个矩阵）

设某坐标系（e1, e2, e3）发生了一次旋转，变成了（e1', e2', e3'）

对于某个固定的向量a（向量不随坐标系旋转），坐标关系:
$$
\left[ \begin{array}{ccc}
	e_1 & e_2 & e_3 
\end{array} \right]
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
\left[ \begin{array}{ccc}
	e_1^\prime & e_2^\prime  & e_3^\prime  
\end{array} \right]
\left[ \begin{array}{c} a_1^\prime  \\ a_2^\prime  \\ a_3^\prime  \end{array} \right]
$$
左乘 $\left[ \begin{array}{c} e^T_1\\ e^T_2\\ e^T_3 \end{array} \right]$ , 得：
$$
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
\left[ \begin{array}{ccc}
	e^T_1e_1^\prime & e^T_1e_2^\prime & e^T_1e_3^\prime \\
	e^T_2e_1^\prime & e^T_2e_2^\prime & e^T_2e_3^\prime \\
	e^T_3e_1^\prime & e^T_3e_2^\prime & e^T_3e_3^\prime
\end{array}\right] 
\left[ \begin{array}{c} a_1^\prime \\ a_2^\prime \\ a_3^\prime \end{array} \right] \triangleq R\vec{a^\prime}
$$
**R 是一个正交矩阵（逆=转置），R的行列式为+1**，称为**旋转矩阵**，则旋转矩阵的**集合**就是**特殊正交群**：$SO(n)= \{ R\in R^{n\times n}|RR^T=I,det(R)=1 \}$ ，n维空间中的旋转，n=3就是三维空间中的旋转

旋转矩阵描述了两个坐标的变换关系（向量没变），比如

* $a_1 = R_{12} a_2$
* 反之：$a_2 = R_{21} a_1$
* 于是：$R_{21}=R_{12}^{-1}=R_{12}^T$
* 进一步，三个坐标系亦有：$a_3=R_{32}a_2=R_{32}R_{21} a1=R_{31}a_1 $

旋转加上平移：$a'=Ra+t$, 两个坐标系的刚体运动可以由R，t 完全描述。

但是如果发生了两次变换：$b=R_1a+t, \ c=R_2b+t_2$

这时：$c = R_2(R_1a+t_1)+t_2$，叠加起来过于复杂

把一次平移和一次旋转合成一个矩阵：
$$
\left[ \begin{array}{c} a^\prime \\ 1 \end{array} \right]=
\left[ \begin{array}{cc} R & t \\ 0^T & 1 \end{array} \right]
\left[ \begin{array}{c} a \\ 1 \end{array} \right] \triangleq
T \left[ \begin{array}{c} a \\ 1 \end{array} \right]
$$
记 $\tilde{a}=\left[ \begin{array}{c} a \\1 \end{array} \right]$ ，这种用四个数表达三维向量的做法称为**齐次坐标**，引入齐次坐标后，旋转和平移可以放入同一个矩阵，称为变换矩阵。那么多次变换就可以写成：
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

### EIGEN

C++矩阵运算库，运行效率高

ubuntu 安装eigen `sudo apt-get install libeigen3-dev`

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

万向锁（Gimbal Lock）：

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

### EIGEN 几何模块

## 第四讲 李群与李代数

对三维世界中刚体运动的估计和优化。

旋转矩阵自身带有约束（正交且行列式为1，非线性约束）。作为优化变量时，会引入额外的约束，使优化变得困难。李代数上可以变成无约束优化

## 4.1 李群李代数基础

三维旋转矩阵构成了特殊正交群
$$
SO(n)= \{ R\in R^{n\times n}|RR^T=I,det(R)=1 \}
$$


三维变换矩阵构成了特殊欧氏群
$$
SE(3)=\left\{ T= \left[ \begin{array}{cc} R & t \\ 0^T & 1 \end{array} \right] \in R^{4\times 4}|R \in SO(3),t \in R^3 \right\}
$$
群：是一种集合加上一种运算代数结构。

记集合为A，运算为·，那么当运算满足以下性质时，称（A，·）成群：

1. 封闭性：$\forall a_1,a_2 \in A,\quad a_1\cdot a_2 \in A$

2. 结合律：$\forall a_1,a_2,a_3 \in A, \quad (a_1·a_2)·a_3 = a_1·（a_2·a_3）$

3. 幺元：$\exists a_0 \in A,\quad s.t. \quad \forall a \in A, \quad a_0·a=a·a_0=a.$

4. 逆：$\forall a \in A, \quad \exists a^{-1} \in A, \quad s.t. \quad a·a^{-1} = a_0$

   封结幺逆（“凤姐咬你”）