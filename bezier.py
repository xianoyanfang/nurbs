import numpy as np
import matplotlib.pyplot as plt

# 通过控制点实现Bezier曲线
def beziercurve_cal(P,k,i,t,is_x):
    if k == 0:
        return P[i][0] if is_x else P[i][1]
    else:
        return (1 - t) * beziercurve_cal(P,k - 1,i,t,is_x) + t * beziercurve_cal(P,k - 1,i + 1,t,is_x)

# 通过控制点实现三次均匀B样条曲线
def factorial(n):
    if n == 1:
        return 1
    else:
        return n * factorial(n - 1)
def BSpline_cal(P):
    Value = []
    a = 1/factorial(3)
    for i  in range(0,len(P) - 3):
        t_array = list(np.arange(0,1,0.01))
        t_array.append(1)
        for t in t_array:
            N03 = a * (1 - t) ** 3
            N13 = a * (4 - 6 * t** 2 + 3 * t ** 3)
            N23 = a * (1 + 3 * t + 3 * t ** 2 - 3 * t ** 3)
            N33 = a * t ** 3
            x = N03 * P[i][0] + N13 * P[i + 1][0] + N23 * P[i + 2][0] + N33 * P[i + 3][0]
            y = N03 * P[i][1] + N13 * P[i + 1][1] + N23 * P[i + 2][1] + N33 * P[i + 3][1]
            Value.append([x,y])
    return Value

if __name__ == "__main__":
    curveType = input("选择Bezier曲线请输入1\n"
                      "选择三次均匀B样条曲线请输入2\n"
                      "请选择 >>> ")
    N = int(input("请输入点个数，必须大于等于4 >>>  "))
    plt.figure()
    plt.plot((-100, -100), (100, 100))
    P = plt.ginput(N)

    if curveType == '1':
        # ====Bezier曲线实现
        t_array = list(np.arange(0, 1, 0.01))
        t_array.append(1)
        Value = []
        for t in t_array:
            Value.append([beziercurve_cal(P,len(P) - 1, 0, t, True), beziercurve_cal(P,len(P) - 1, 0, t, False)])
        # ====Bezier曲线实现
    elif curveType == '2':
        # ====三次均匀B样条曲线
            Value = BSpline_cal(P)
        # ====三次均匀B样条曲线

    X = [i[0] for i in Value]
    Y = [i[1] for i in Value]
    plt.plot(X, Y, "r")
    scatter_x = [i[0] for i in P]
    scatter_y = [i[1] for i in P]
    plt.scatter(scatter_x, scatter_y)
    plt.plot(scatter_x, scatter_y, 'b')

    i = 1
    for point in P:
        plt.text(point[0], point[1], "P" + str(i))
        i += 1
    plt.show()