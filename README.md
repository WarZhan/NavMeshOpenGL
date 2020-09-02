# NavMeshOpenGL
凸多边形寻路算法
博文地址：[https://blog.csdn.net/zhanxi1992/article/details/82725185](https://blog.csdn.net/zhanxi1992/article/details/82725185)
## 一、导入数据生成NavMesh导航轮廓

1、由文本导入数据，数据要求：可走轮廓的最外围为顺时针方向，障碍物为逆时针方向，顶点不可有重复的，各障碍物不可叠加；

2、使用扫描线算法划分三角形，生成多边形轮廓；

3、将生成轮廓顶点信息转换为逆时针存储。

## 二、NavMesh导航网格的构建

1、在第一步生成的基础上构建多边形轮廓对象；

2、根据输入的轮廓三角形化，划分为紧凑拼接的三角形；

3、合并三角形，合并中先判断合成之后的多边形是否为凸多边形，是则合并，不是则跳过。

4、生成一个rcPolyMesh对象，包含顶点信息及多边形信息，npolys为多边形的数量，polys为多边形的数据，存放的为对应的多边形定点索引，nvp为多边形最大的顶点数。

## 三、寻路算法的实现（Dijkstra）（红色）

1、根据生成的rcPolyMesh,计算邻接信息（int二维数组），生成邻接矩阵，邻接的权值用所在多边形的最低点和最高点所连接对角线的长度。

2、鼠标左键点击确定起点，右键点击确定终点，点击鼠标的中间开始寻路。

3、根据起点和终点的位置确定两点分别在的多边形，使用顶点与多边形的边进行叉乘运算，若都在边得左侧，则点位于该多边形中。

4、根据Dijkstra算法得出一条最短路，找出所有邻接多边形的公共边，使用拐点发优化路径。

 

## 四、优化 A* (绿色)

1、根据生成的rcPolyMesh，计算邻接信息（bool二维数组）和多边形重心位置（Vec3），重心计算方式为；
<center>![重心公式](https://github.com/WarZhan/NavMeshOpenGL/blob/master/Res/1.png)</center>


2、鼠标左键点击确定起点，右键点击确定终点，点击鼠标的中间开始寻路；

3、根据起点和终点的位置确定两点分别在的多边形，使用顶点与多边形的边进行叉乘运算，若都在边得左侧，则点位于该多边形中；

4、A* 中所用估值函数为位置上两方向的增量（dx+dy），两多边形间的花费近似为两多边形重心的位置，得到路径后用拐点法优化。

<center>![demo寻路图片1](https://github.com/WarZhan/NavMeshOpenGL/blob/master/Res/2.png)</center>
<center>![demo寻路图片2](https://github.com/WarZhan/NavMeshOpenGL/blob/master/Res/3.png)</center>
<center>![demo寻路图片3](https://github.com/WarZhan/NavMeshOpenGL/blob/master/Res/4.png)</center>
<center>![demo寻路图片4](https://github.com/WarZhan/NavMeshOpenGL/blob/master/Res/5.png)</center>

**注：两种寻路算法在出现扁长形多边形时，可能搜出的路径不是最优,当两种算法路径一样时，红色的被覆盖只能看见绿色的。**

## 参考资料

1. 多边形生成三角形：[http://sites-final.uclouvain.be/mema/Poly2Tri/poly2tri.html](http://sites-final.uclouvain.be/mema/Poly2Tri/poly2tri.html)

2. 凸多边形网格构建：recastnavigation [https://github.com/memononen/recastnavigation](https://github.com/memononen/recastnavigation)

3. 凸多边形网络三角形划分：[http://www.critterai.org/nmgen_polygen](http://www.critterai.org/nmgen_polygen)

4. 路径优化拐点法： [http://www.navpower.com/gdc2006_miles_david_pathplanning.ppt](http://www.navpower.com/gdc2006_miles_david_pathplanning.ppt)
