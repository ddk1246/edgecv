import PID
import matplotlib.pyplot as plt
import math
plt.figure(1)  # 创建图表1
plt.figure(2)  # 创建图表2


# 测试PID程序
def TestPID(P, I, D):
    IncrementalPid = PID.IncrementalPID(P, I, D)
    PositionalPid = PID.PositionalPID(P, I, D)
    IncrementalXaxis = [0]
    IncrementalYaxis = [0]
    PositionalXaxis = [0]
    PositionalYaxis = [0]

    x=range(1,500)
    y=[5*math.sin(i*0.1)+10 for i in x]

    for i in x:
        # 增量式
        IncrementalPid.SetStepSignal(y[i-1])
        IncrementalPid.SetInertiaTime(3, 0.1)
        IncrementalYaxis.append(IncrementalPid.SystemOutput)
        IncrementalXaxis.append(i)

        # 位置式
        PositionalPid.SetStepSignal(y[i-1])
        PositionalPid.SetInertiaTime(3, 0.1)
        PositionalYaxis.append(PositionalPid.SystemOutput)
        PositionalXaxis.append(i)

    plt.figure(1)  # 选择图表1
    plt.plot(IncrementalXaxis, IncrementalYaxis, 'r',x,y)
    plt.xlim(0, 120)
    plt.ylim(0, 140)
    plt.title("IncrementalPID")

    plt.figure(2)  # 选择图表2
    plt.plot(PositionalXaxis, PositionalYaxis, 'b')
    plt.xlim(0, 120)
    plt.ylim(0, 140)
    plt.title("PositionalPID")

    plt.show()


if __name__ == "__main__":
    TestPID(4.5, 0.5, 0.1)


