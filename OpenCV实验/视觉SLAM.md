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
	e_1\verb|'| & e_2\verb|'|  & e_3\verb|'|  
\end{array} \right]
\left[ \begin{array}{c} a_1\verb|'|  \\ a_2\verb|'|  \\ a_3\verb|'|  \end{array} \right]
$$
左乘 $\left[ \begin{array}{c} e^T_1\\ e^T_2\\ e^T_3 \end{array} \right]$ , 得：
$$
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] =
\left[ \begin{array}{ccc}
	e^T_1e_1\verb|'| & e^T_1e_2\verb|'| & e^T_1e_3\verb|'| \\
	e^T_2e_1\verb|'| & e^T_2e_2\verb|'| & e^T_2e_3\verb|'| \\
	e^T_3e_1\verb|'| & e^T_3e_2\verb|'| & e^T_3e_3\verb|'| 	
\end{array}\right] 
\left[ \begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array} \right] \triangleq R\vec{a\verb|'|}
$$
R 是一个正交矩阵（逆=转置），R的行列式为+1，称为**旋转矩阵**，则旋转矩阵的**集合**就是：$SO(n)= \{ R\in R^{n\times n}|RR^T=I,det(R)=1 \}$ ，n维空间中的旋转，n=3就是三维空间中的旋转

旋转矩阵描述了两个坐标的变换关系，比如

* $a_1 = R_{12} a_2$
* 反之：$a_2 = R_{21} a_1$
* 于是：$R_{21}=R_{12}^{-1}=R_{12}^T$
* 进一步，三个坐标系亦有：$a_3=R_{32}a_2=R_{32}R_{21} a1=R_{31}a_1 $

加上平移：$a'=Ra+t$, 两个坐标系的刚体运动可以由R，t完全描述。

但是如果发生了两次变换：$b=R_1a+t, \ c=R_2b+t_2$

这时：$c = R_2(R_1a+t_1)+t_2$，叠加起来过于复杂