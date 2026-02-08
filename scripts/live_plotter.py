#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math

class LiveYawPlotter:
    def __init__(self):
        rospy.init_node('live_yaw_plotter', anonymous=True)
        
        # Buffers for plotting (increased for full mission history)
        self.max_points = 50000 
        self.times = []
        self.raw_yaws = []
        self.ekf_yaws = []
        self.truth_yaws = []
        
        self.start_time = rospy.get_time()
        
        rospy.Subscriber('yaw_comparison_data', Float32MultiArray, self.callback)
        
        # Setup Plot (Premium Light High-Contrast Theme)
        plt.style.use('default') 
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.fig.patch.set_facecolor('#ffffff')
        self.ax.set_facecolor('#ffffff')
        
        self.line_raw, = self.ax.plot([], [], color='#d32f2f', label='Raw (INS)', alpha=0.6, lw=1)
        self.line_ekf, = self.ax.plot([], [], color='#2e7d32', label='EKF (Standard)', lw=2.5)
        self.line_truth, = self.ax.plot([], [], color='#0277bd', label='Ground Truth (Course)', lw=1.5, ls='--')
        
        # DEGREE LIMITS: -190 to 190
        self.ax.set_ylim(-190, 190)
        self.ax.set_title("LIVE SENSOR FUSION DIAGNOSTICS (DEGREES)", fontsize=14, fontweight='bold', pad=20, color='#222222')
        self.ax.set_xlabel("MISSION ELAPSED TIME (S)", fontsize=10, color='#555555')
        self.ax.set_ylabel("HEADING (DEGREES)", fontsize=10, color='#555555')
        
        self.ax.legend(loc='upper right', frameon=True, facecolor='#f5f5f5', edgecolor='#cccccc', fontsize='small')
        self.ax.grid(True, color='#dddddd', linestyle=':', alpha=0.8)
        
        # Axis lines
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['left'].set_color('#aaaaaa')
        self.ax.spines['bottom'].set_color('#aaaaaa')

    def callback(self, msg):
        if len(msg.data) < 3: return
        
        t = rospy.get_time() - self.start_time
        self.times.append(t)
        self.raw_yaws.append(msg.data[0])
        self.ekf_yaws.append(msg.data[1])
        self.truth_yaws.append(msg.data[2])
        
        # Keep buffer size stable
        if len(self.times) > self.max_points:
            self.times.pop(0)
            self.raw_yaws.pop(0)
            self.ekf_yaws.pop(0)
            self.truth_yaws.pop(0)

    def update_plot(self, frame):
        if not self.times: return self.line_raw, self.line_ekf, self.line_truth
        
        self.line_raw.set_data(self.times, self.raw_yaws)
        self.line_ekf.set_data(self.times, self.ekf_yaws)
        self.line_truth.set_data(self.times, self.truth_yaws)
        
        # Dynamic X-axis
        if len(self.times) > 1:
            self.ax.set_xlim(self.times[0], self.times[-1])
            
        return self.line_raw, self.line_ekf, self.line_truth

    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.show()

if __name__ == '__main__':
    try:
        plotter = LiveYawPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
