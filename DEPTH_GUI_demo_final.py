import tkinter as tk
from tkinter import ttk
import threading
import time
import random
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

grid_df = pd.read_csv("grid_min.csv", sep=";")

class DepthDemoGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Depth Control System - Unified GUI")

        self.root.geometry("800x400")

        self.mode_var = tk.StringVar(value="AUTO")
        tk.Label(self.root, text="Mode:").grid(row=0, column=0, sticky="e", padx=5, pady=2)
        mode_frame = tk.Frame(self.root)
        mode_frame.grid(row=0, column=1, sticky="w")
        tk.Radiobutton(mode_frame, text="Auto", variable=self.mode_var, value="AUTO").pack(side="left")
        tk.Radiobutton(mode_frame, text="Manual", variable=self.mode_var, value="MANUAL").pack(side="left")

        self.labels = {}
        for i, key in enumerate(["Time", "Longitude", "Latitude", "Max Depth", "Current Depth"], start=1):
            tk.Label(self.root, text=f"{key}:").grid(row=i, column=0, sticky='e', padx=5, pady=2)
            var = tk.StringVar()
            self.labels[key] = var
            tk.Label(self.root, textvariable=var).grid(row=i, column=1, sticky='w', padx=5, pady=2)

        # 图像区域
        self.fig, self.ax = plt.subplots(figsize=(5, 3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=2, rowspan=6, padx=10, pady=10)
        self.ax.set_ylabel("Depth (m)")
        self.ax.invert_yaxis()
        self.max_depth_history = []
        self.cur_depth_history = []
        self.time_history = []

        self.running = True
        self.root.after(1000, self.fake_data_loop)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def update_gui(self, data):
        for k, v in data.items():
            if k in self.labels:
                self.labels[k].set(v)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_ylabel("Depth (m)")
        self.ax.invert_yaxis()
        if len(self.time_history) > 0:
            self.ax.plot(self.time_history, self.max_depth_history, label='Seabed Depth', color='black')
            self.ax.plot(self.time_history, self.cur_depth_history, label='Camera Depth', color='gold')
            self.ax.set_xticklabels(self.time_history, rotation=45, ha='right')
            self.ax.legend()
        self.canvas.draw()

    def fake_data_loop(self):
        if not self.running:
            return
        now = datetime.now().strftime("%H:%M:%S")
        mode = self.mode_var.get()
        row = grid_df.sample(1).iloc[0]
        lon = (row['min_lon'] + row['max_lon']) / 2
        lat = (row['min_lat'] + row['max_lat']) / 2
        seabed = round(row['min_depth'], 2)
        current_depth = round(seabed + 1.5 + random.uniform(-0.2, 0.2), 2)

        self.update_gui({
            "Time": now,
            "Longitude": f"{lon:.5f}",
            "Latitude": f"{lat:.5f}",
            "Max Depth": f"{seabed:.2f}",
            "Current Depth": f"{current_depth:.2f}"
        })

        # 更新历史用于折线图
        self.time_history.append(now)
        self.max_depth_history.append(seabed)
        self.cur_depth_history.append(current_depth)
        if len(self.time_history) > 20:
            self.time_history.pop(0)
            self.max_depth_history.pop(0)
            self.cur_depth_history.pop(0)

        self.update_plot()
        self.root.after(1000, self.fake_data_loop)

    def on_close(self):
        self.running = False
        self.root.destroy()

if __name__ == "__main__":
    DepthDemoGUI()
