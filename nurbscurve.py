import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 这个可以绘制三维图形

# Beizer Curve 接口
# 简述Bezier曲线，具有分割特性
# 具有下面的递推公式，pi,0为控制点
# pi_k = { pi,k = 0;
#          (1 - t) * pi_(k-1) + t * p(i+1)_(k - 1)
def bezierCurve(arrayList,i,k,t):
    if k == 0 :
        return arrayList[i]
    else:
        return (1 - t) * bezierCurve(arrayList,i,k - 1,t) + t * bezierCurve(arrayList,i + 1,k - 1,t)

# Bezier Surface 接口

# 均匀三次样条曲线，
def threeBSplineCurve(P):
    K = 3
    i = 0
    M = 1/6 * np.array([[-1,3,-3,1],[3,-6,3,0],[-3,0,3,0],[1,4,1,0]])
    t = np.arange(0,1.01,0.01)
    one = np.ones(len(t))
    U = np.array([t**3,t**2,t,one]).T
    tP = np.array(P)
    return np.dot(np.dot(U,M),tP)

# 由经过均匀三次样条曲线逆推控制点
def invThreeBSplineCurve(P):
    P = np.array(P)
    N = len(P)
    P0_diff = P[1] - P[0]
    P1_diff = P[N - 1] - P[N - 2]
    M = np.zeros((N+2,N+2))
    TP = []
    for i in np.arange(0,N+2):
        if i == 0:
            M[i,0:3] = [-1,0,1]
            TP.append(2 * P0_diff)
        elif i == (N+1):
            M[i,(N-1):(N+2)] = [-1,0,1]
            TP.append(2 * P1_diff)
        else:
            M[i,(i -1):(i+2)] = [1,4,1]
            TP.append(6 * P[i-1])
    return np.dot(np.linalg.inv(M),np.array(TP))

# 非均匀B样条曲线
def baseN(N,KK,k,U,i,u):
    if k < 0 or i > (N - 1):
        return 0
    if k == 0 :
        if u >= U[i] and u <= U[i+1]:
            return 1
        else :
            return 0
    a = (u- U[i])
    b = U[i + KK] - U[i]
    c = (U[i + KK + 1] - u)
    d = U[i + KK + 1] - U[i + 1]
    return (0 if b==0 else (a/b)) * baseN(N,KK,k - 1,U,i,u) + (0 if d==0 else (c/d)) * baseN(N,KK,k - 1,U,i + 1,u)

# 其实前面的并没有错误，只不过是PPT里面关于非均匀样条曲线的定义中有部分不明确的地方，非均匀样条曲线实际上的权值应该为1，即还是需要除以SUM（N_i）,不然绘制出来的曲线很奇怪,
# 像nurbs曲线实际上就是在此基础上为每个控制点赋予一个权重
def BSpline(P,K,U):
    pointCount = len(P)
    RPX = []
    RPY = []
    # 确定参数为u时的值
    for u in np.arange(0,U[-1],0.01):
        denominator = 0
        for i in range(pointCount):
            denominator += baseN(pointCount, K, K, U, i, u)
        resultPointX = 0
        resultPointY = 0
        for i in range(pointCount):
            bn = baseN(pointCount, K, K, U, i, u)
            resultPointX += P[i][0] * bn/denominator
            resultPointY += P[i][1] * bn/denominator
        RPX.append(resultPointX)
        RPY.append(resultPointY)
    return [RPX,RPY]

if __name__ == "__main__":
    ## 绘制Bezier曲线
    # while True:
    #     N = int(input("请输入控制该曲线的控制点数量 << "))
    #     # Bezier曲线的阶次与曲线的控制点相关，K = N - 1，要将其推广到任意阶次
    #     K = N - 1
    #     plt.figure()
    #     plt.plot((-100, -100), (100, 100))
    #     P = plt.ginput(N)
    #     P_X = [p[0] for p in P]
    #     P_Y = [p[1] for p in P]
    #     t_array = list(np.arange(0,1.01,0.01))
    #     R_x = []
    #     R_y = []
    #     for t in t_array:
    #         # 由bezier曲线的分割特性可知最终i应该为0
    #         R_x.append(bezierCurve(P_X,0,K,t))
    #         R_y.append(bezierCurve(P_Y,0,K,t))
    #     scatter_x = [i[0] for i in P]
    #     scatter_y = [i[1] for i in P]
    #     plt.plot(scatter_x, scatter_y, 'b')
    #     plt.scatter(scatter_x, scatter_y)
    #     plt.plot(R_x,R_y, "r")
    #     i = 1
    #     for point in P:
    #         plt.text(point[0], point[1], "P" + str(i))
    #         i += 1
    #     plt.show()

    # # 绘制均匀三次样条曲线
    # while True:
    #     N = int(input("请输入均匀三次样条曲线的控制点个数："))
    #     plt.figure()
    #     plt.plot((-100, -100), (100, 100))
    #     P = plt.ginput(N)
    #     scatter_x = [i[0] for i in P]
    #     scatter_y = [i[1] for i in P]
    #     plt.scatter(scatter_x, scatter_y)
    #     plt.plot(scatter_x,scatter_y,'g')
    #     K = 3
    #     times = N - K
    #     i = 0
    #     rP = []
    #     while times:
    #         tP = P[i:(i+K+1)]
    #         rP.extend(threeBSplineCurve(tP))
    #         times -= 1
    #         i += 1
    #     rX = [i[0] for i in rP]
    #     rY = [i[1] for i in rP]
    #     plt.plot(rX,rY,'r')
    #     plt.show()

    # # 通过确定均匀三次B样条曲线上的点逆推出控制点位置
    # while True:
    #     N = int(input("请输入均匀三次样条曲线上的点个数:"))
    #     plt.figure()
    #     plt.plot((-100, -100), (100, 100))
    #     P = plt.ginput(N)
    #     scatter_x = [i[0] for i in P]
    #     scatter_y = [i[1] for i in P]
    #     plt.scatter(scatter_x, scatter_y)
    #     controlPoints = invThreeBSplineCurve(P)
    #     scatter_x = [i[0] for i in controlPoints]
    #     scatter_y = [i[1] for i in controlPoints]
    #     plt.scatter(scatter_x, scatter_y)
    #     plt.plot(scatter_x,scatter_y,'g')
    #     CN = len(controlPoints)
    #     K = 3
    #     times =CN - K
    #     i = 0
    #     rP = []
    #     while times:
    #         tP = controlPoints[i:(i + K + 1)]
    #         rP.extend(threeBSplineCurve(tP))
    #         times -= 1
    #         i+=1
    #     rX = [i[0] for i in rP]
    #     rY = [i[1] for i in rP]
    #     plt.plot(rX,rY,'r')
    #     plt.show()

    # 非均匀样条曲线
    while True:
        N = int(input("请输入点数："))
        K = int(input("请输入nurbs曲线次数："))
        U = [float(x) for x in input("请输入nurbs曲线的" + str(N + K + 1) + "个节点(请用空格分隔开)：").split(" ")]
        plt.figure()
        plt.axis([-100, 100, -100, 100])
        P = plt.ginput(N)
        X,Y = BSpline(P,K,U)
        plt.plot(X, Y, "r")
        controlPoint_X = [point[0] for point in P]
        controlPoint_Y = [point[1] for point in P]
        plt.scatter(controlPoint_X, controlPoint_Y)
        plt.plot(controlPoint_X, controlPoint_Y, 'b')
        plt.show()