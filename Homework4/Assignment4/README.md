-----作业说明-----
[已完成] 提交的格式正确，包含所有必须的文件。代码可以编译和运行。
[已完成] De Casteljau 算法：
对于给定的控制点，你的代码能够产生正确的 Bézier 曲线。
参考./images/my_bezier_curve_noAA.png
[已完成] 奖励分数：
实现对 Bézier 曲线的反走样。(对于一个曲线上的点，不只把它对应于一个像
素，你需要根据到像素中心的距离来考虑与它相邻的像素的颜色。)
参考./images/my_bezier_curve_AA.png

-----函数说明-----
在recursive_bezier 中构建了递归的de Casteljau 的算法来计算各个控制点通过给定的t推出作为曲线的点的位置
在LinearLerp中通过t返回了a，b两点之间的插值
在bezier 中通过调用 recursive_bezier 并且通过for loop模拟了t【0,1】的step in, 最后在Bézier 曲线的基础上做了Bilinear反走样


-----特别说明-----
修改了naive-bezier从而让反走样效果图的效果更好
修改了main从而直接产出aa和非aa的效果图

mike feng 
2021-07-22