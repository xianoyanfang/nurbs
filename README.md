## nurbs
 bezier/nurbs curve
### Bezier Curve
beizer曲线的公式是采用B_{i,n}(t) = B_{i,n - 1}(t) * (1 - t) + B_{i + 1, n - 1}(t) * t进行递归计算，其中B表示初始控制点，t表示当前曲线阈值，n表示曲线阶次，i表示第几个控制点,通过这个曲线即可确定由用户在绘图区域点击的点列，并由这些点列生成N阶Bezier曲线
