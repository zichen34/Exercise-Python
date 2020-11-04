from https://www.cnblogs.com/Jessica-jie/p/8622449.html 博客园

## OpenCV提供了两种Matching方式：

### 1. Brute-force matcher (cv::BFMatcher)

* 暴力方法找到点集1中每个descriptor 在点集2中距离最近的descriptor；找寻到的距离最小就认为匹配。
* 浮点描述子-欧氏距离；二进制描述符-汉明距离。
* 详细描述：**在第一幅图像中选取一个关键点然后依次与第二幅图像的每个关键点进行（描述符）距离测试，最后返回距离最近的关键点**

`cv::BFMatcher(int normType=NORM_L2, bool crossCheck=false)`

1. **normType**: 它是用来**指定要使用的距离测试类型**, 默认值为**cv2.Norm_L2**,这很适合SIFT和SURF等（c2.NORM_L1也可）。
   对于使用二进制描述符的ORB、BRIEF和BRISK算法等，要使用**cv2.NORM_HAMMING**，这样就会返回两个测试对象之间的汉明距离。
   如果ORB算法的参数设置为WTA_K==3或4，normType就应该设置成**cv2.NORM_HAMMING2**。

2. **crossCheck**：默认值为False。如果设置为True，匹配条件就会**更加严格**，只有到A中的第i个特征点与B中的第j个特征点距离最近，并且B中的第j个特征点到A中的第i个特征点也是最近时才会返回最佳匹配(i,j)，即这两个特征点要互相匹配才行。

3. BFMatcher对象有两个方法**BFMatcher.match()**和**BFMatcher.knnMatch()**。第一个方法会返回**最佳匹配**。第二个方法为**每个关键点返回k个最佳匹配**，其中k是由用户设定的。

   **cv2.drawMatches()**来绘制匹配的点，它会将两幅图像先水平排列，然后在最佳匹配的点之间绘制直线。

   如果前面使用的是BFMatcher.knnMatch()，现在可以使用函数**cv2.drawMatchsKnn**为每个关键点和它的个最佳匹配点绘制匹配线，如果要选择性绘制就要给函数传入一个掩模。

一般，点集1称为 **train set （训练集）**的对应**模板图像**，点集2称为 **query set（查询集）**的对应查找模板图的**目标图像**。

为了提高检测速度，你可以调用matching函数前，先训练一个matcher。训练过程可以首先使用cv::FlannBasedMatcher来优化，为descriptor建立索引树，这种操作将在匹配大量数据时发挥巨大作用。

而Brute-force matcher在这个过程并不进行操作，它只是将train descriptors保存在内存中。



### 2. Flann-based matcher (cv::FlannBasedMatcher)

* 快速最近邻搜索算法寻找（用快速的第三方库近似最近邻搜索算法）

* 是一个对大数据集和高维特征进行最近邻搜索的算法的集合，在面对大数据集时它的效果要好于BFMatcher。

* 使用FLANN匹配需要传入两个字典参数：

  1. 一个参数是IndexParams，
     1. 对于SIFT和SURF，可以传入参数index_params=dict(algorithm=FLANN_INDEX_KDTREE, trees=5)。
     2. 对于ORB，可以传入参数index_params=dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)。

  2. 第二个参数是SearchParams，可以传入参数search_params=dict(checks=100)，它来指定递归遍历的次数，值越高结果越准确，但是消耗的时间也越多。

