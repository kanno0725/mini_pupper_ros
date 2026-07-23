import numpy as np


class Configuration:
    """リアクティブ回避ノードの調整パラメータ。

    値をここに集約しておくと、ノード本体を触らずに挙動を調整できる。
    """

    def __init__(self):
        # 制御ループの周期 [s]（0.1 = 10Hz）
        self.timer_period = 0.1

        # 前方の基準向き [rad]。lidar 座標系での正面を表す。
        # lidar は 90°回して取り付けているため、lidar の 0°がロボット正面と
        # 一致するとは限らない。sim で箱を正面に置き、front_min が最小になる
        # 向きへ実測で合わせる（多くは ±np.pi/2 付近）。
        self.front_rad = 0.0

        # 各セクタの角度幅 [rad]（np.pi/3 = 60° → 前方は ±30°）
        self.sector_width_rad = np.pi / 3

        # これより近い障害物があれば「ふさがっている」と判定する距離 [m]
        self.obstacle_threshold = 2

        # 前進速度 [m/s]。下流の max_x_velocity=0.20 以下にする
        self.forward_speed = 0.05

        # その場旋回の角速度 [rad/s]
        self.turn_speed = 2
