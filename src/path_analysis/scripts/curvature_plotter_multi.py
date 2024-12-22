#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # 根据您的系统配置选择合适的后端
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from threading import Lock

class CurvaturePlotterMulti:
    def __init__(self):
        rospy.init_node('curvature_plotter_multi', anonymous=True)

        # 订阅三个绝对路径话题
        self.path_subscribers = {
            'plan': rospy.Subscriber('/move_base/PathPlanner/plan', Path, self.path_callback, callback_args='plan'),
            'plan1': rospy.Subscriber('/move_base/PathPlanner/plan1', Path, self.path_callback, callback_args='plan1'),
            'plan2': rospy.Subscriber('/move_base/PathPlanner/plan2', Path, self.path_callback, callback_args='plan2')
        }

        # 存储路径数据
        self.paths = {
            'plan': None,
            'plan1': None,
            'plan2': None
        }

        # 标记路径是否已更新
        self.updated = {
            'plan': False,
            'plan1': False,
            'plan2': False
        }

        # 锁用于线程安全
        self.lock = Lock()

        # 初始化Matplotlib图形
        plt.ion()  # 开启交互模式
        self.fig, self.ax = plt.subplots(figsize=(14, 8))  # 增大图形尺寸

        # 定义不同的线型和标记
        line_styles = {
            'plan': {'color': 'blue', 'linestyle': '-', 'marker': None, 'label': 'A-star'},               # 实线
            'plan1': {'color': 'green', 'linestyle': '--', 'marker': None, 'label': 'B-Spline-A-star'}, # 虚线
            'plan2': {'color': 'red', 'linestyle': '-', 'marker': '^', 'markersize': 8, 'label': 'Improved-A-star'}  # 实线 + 三角形标记
        }

        # 创建线对象
        self.lines = {}
        for plan_name, style in line_styles.items():
            self.lines[plan_name] = self.ax.plot([], [], label=style['label'],
                                                color=style['color'],
                                                linestyle=style['linestyle'],
                                                marker=style['marker'],
                                                markersize=style.get('markersize', 5),
                                                linewidth=2)[0]

        # 设置坐标轴标签和标题
        self.ax.set_xlabel('Arc Length (s) [m]', fontsize=14)
        self.ax.set_ylabel('Curvature κ(s) [1/m]', fontsize=14)
        self.ax.set_title('Curvature vs Arc Length for Multiple Paths', fontsize=18)

        # 设置图例，并增大字体大小
        self.ax.legend(fontsize=16)

        # 优化网格样式
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

        # 设置曲率轴的主刻度间隔为0.1，次刻度为0.05
        self.ax.yaxis.set_major_locator(MultipleLocator(0.1))
        self.ax.yaxis.set_minor_locator(MultipleLocator(0.05))
        self.ax.tick_params(axis='both', which='major', labelsize=12)
        self.ax.tick_params(axis='both', which='minor', labelsize=10)

        # 自动调整子图参数以填充整个图像区域
        plt.tight_layout()

        plt.show()

        rospy.loginfo("Curvature Plotter Multi Node Initialized.")

    def path_callback(self, msg, plan_name):
        with self.lock:
            rospy.loginfo(f"Received path for {plan_name} with {len(msg.poses)} poses.")
            self.paths[plan_name] = msg.poses
            self.updated[plan_name] = True

            # 打印前3个路径点坐标
            for i, pose in enumerate(msg.poses[:3]):
                x = pose.pose.position.x
                y = pose.pose.position.y
                rospy.loginfo(f"{plan_name} Pose {i}: x = {x:.2f} m, y = {y:.2f} m")

    def compute_arc_length(self, x, y):
        """
        计算每个点的累积弧长 s。
        """
        s = [0.0]
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            ds = np.sqrt(dx**2 + dy**2)
            s.append(s[-1] + ds)
        return np.array(s)

    def compute_curvature_triangle(self, x, y):
        """
        使用基于三角形的方法计算曲率 κ。
        对于每三个连续点 (P1, P2, P3)，计算它们形成的圆的曲率。
        """
        curvature = np.zeros(len(x))
        for i in range(1, len(x)-1):
            x1, y1 = x[i-1], y[i-1]
            x2, y2 = x[i], y[i]
            x3, y3 = x[i+1], y[i+1]

            # 计算边长
            a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            b = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
            c = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)

            # 计算半周长
            s_tri = (a + b + c) / 2

            # 计算三角形面积（海伦公式）
            area_square = s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c)
            area_square = max(area_square, 0)
            area = np.sqrt(area_square) if area_square > 0 else 0

            if area == 0:
                curvature[i] = 0.0
            else:
                # 计算圆的半径
                R = (a * b * c) / (4 * area)
                curvature[i] = 1 / R

        # 处理曲率边界点
        if len(curvature) >= 2:
            curvature[0] = curvature[1]
            curvature[-1] = curvature[-2]

        return curvature

    def update_plot(self):
        with self.lock:
            updated_any = False
            for plan_name, poses in self.paths.items():
                if poses is None:
                    continue
                if self.updated[plan_name]:
                    if len(poses) < 3:
                        rospy.logwarn(f"{plan_name} 路径点不足，无法计算曲率。需要至少3个点。")
                        self.lines[plan_name].set_data([], [])
                        self.updated[plan_name] = False
                        continue

                    # 提取x, y 并放大20倍
                    x = [pose.pose.position.x * 20 for pose in poses]
                    y = [pose.pose.position.y * 20 for pose in poses]

                    # 计算弧长 s
                    s = self.compute_arc_length(x, y)

                    # 计算曲率 κ（基于三角形的方法）
                    curvature = self.compute_curvature_triangle(x, y)

                    # 更新图形
                    self.lines[plan_name].set_data(s, curvature)

                    # 标记为已更新
                    self.updated[plan_name] = False
                    updated_any = True

        if updated_any:
            with self.lock:
                # 更新坐标轴范围（保持x轴自动缩放，y轴固定刻度间隔为0.1）
                self.ax.relim()
                self.ax.autoscale_view(scalex=True, scaley=False)  # 只自动缩放x轴，保持y轴固定

            # Redraw the figure
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)  # 确保Matplotlib处理事件

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

if __name__ == '__main__':
    try:
        plotter = CurvaturePlotterMulti()
        plotter.spin()
    except rospy.ROSInterruptException:
        pass
