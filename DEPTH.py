import pandas as pd
import numpy as np
from gnss_reader import GNSSReader, GNSSReaderDual
from gnss_reader import GGAData, RMCData, VTGData, GSVData, HDTData
import threading
from rtree import index
import RPi.GPIO as GPIO
import time
import xlsxwriter
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# ----------------------
# PID Controller Class
# ----------------------
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, target, current):
        error = target - current
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1e-5
        self.integral += error * dt
        self.integral = np.clip(self.integral, -10/self.Ki, 10/self.Ki)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        self.last_time = current_time
        return output

# ----------------------
# Motor Controller Class
# ----------------------
class MotorController:
    def __init__(self, up_pin=17, down_pin=27, sensor_up=22, sensor_down=23):
        GPIO.setmode(GPIO.BCM)
        self.up_pin = up_pin
        self.down_pin = down_pin
        self.sensor_up = sensor_up
        self.sensor_down = sensor_down
        GPIO.setup([up_pin, down_pin], GPIO.OUT)
        GPIO.setup([sensor_up, sensor_down], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.motor_stop()

    def motor_up(self):
        self.motor_stop()
        if GPIO.input(self.sensor_up) == GPIO.HIGH:
            GPIO.output(self.up_pin, GPIO.HIGH)
        else:
            print("[WARNING] Upper limit reached!")

    def motor_down(self):
        self.motor_stop()
        if GPIO.input(self.sensor_down) == GPIO.HIGH:
            GPIO.output(self.down_pin, GPIO.HIGH)
        else:
            print("[WARNING] Lower limit reached!")

    def motor_stop(self):
        GPIO.output([self.up_pin, self.down_pin], GPIO.LOW)

# ----------------------
# Depth Control System
# ----------------------
class DepthControlSystem:
    def __init__(self, mode_switch_pin=5):
        self.motor = MotorController()
        self.gnss = GNSSReader(baudrate=9600)  # use GNSSReaderDual() for dual setups
        self.df = pd.read_csv("grid_min.csv", sep=";")
        self.build_spatial_index()

        self.current_depth = 0.0
        self.target_offset = 1.5
        self.pid = PIDController(Kp=0.8, Ki=0.05, Kd=0.2)
        self.mode_switch_pin = mode_switch_pin
        GPIO.setup(mode_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.control_mode = 'auto'

        self.pulley_turns = 0
        self.pulley_diameter = 0.1  # TODO: Replace with actual value
        self.half_diagonal = 1.0    # TODO: Replace with actual value

        self.top_sensor_pin = 24
        GPIO.setup(self.top_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.create_excel_logger()
        self.init_gui()

    def build_spatial_index(self):
        self.idx = index.Index()
        for i, row in self.df.iterrows():
            self.idx.insert(i, (row['min_lon'], row['min_lat'], row['max_lon'], row['max_lat']))

    def get_seabed_depth(self, lon, lat):
        matches = list(self.idx.intersection((lon, lat, lon, lat)))
        return self.df.iloc[matches[0]]['min_depth'] if matches else None

    def check_mode(self):
        first_read = GPIO.input(self.mode_switch_pin)
        time.sleep(0.05)
        second_read = GPIO.input(self.mode_switch_pin)
        if first_read == second_read:
            return 'manual' if first_read == GPIO.LOW else 'auto'
        else:
            return self.control_mode

    def is_at_top(self):
        return GPIO.input(self.top_sensor_pin) == GPIO.LOW

    def compute_rope_length(self, turns, diameter, half_diag):
        return np.sqrt((turns * np.pi * diameter + half_diag)**2 - half_diag**2)

    def create_excel_logger(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.excel_filename = f"DepthTrial_{timestamp}.xlsx"
        self.excel_workbook = xlsxwriter.Workbook(self.excel_filename)
        self.excel_sheet = self.excel_workbook.add_worksheet()
        self.excel_sheet.write_row(0, 0, ["Time", "Longitude", "Latitude", "Seabed Depth", "Target Depth", "Current Depth"])
        self.log_row = 1

    def log_data(self, lon, lat, seabed, target_depth, current_depth):
        t = datetime.now().strftime("%H:%M:%S")
        self.excel_sheet.write_row(self.log_row, 0, [t, lon, lat, seabed, target_depth, current_depth])
        self.log_row += 1

    def init_gui(self):
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

        self.fig, self.ax = plt.subplots(figsize=(5, 3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=2, rowspan=6, padx=10, pady=10)
        self.ax.set_ylabel("Depth (m)")
        self.ax.invert_yaxis()
        self.max_depth_history = []
        self.cur_depth_history = []
        self.time_history = []

        threading.Thread(target=self.auto_control_loop, daemon=True).start()
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
            self.ax.set_xticks(range(len(self.time_history)))
            self.ax.set_xticklabels(self.time_history, rotation=45, ha='right')
            self.ax.legend()
        self.canvas.draw()

    def auto_control_loop(self):
        for msg in self.gnss.read_sentences():
        self.control_mode = self.mode_var.get()
        now_time = datetime.now().strftime("%H:%M:%S")

        if isinstance(msg, GGAData):
            lon, lat = msg.longitude, msg.latitude
            seabed = self.get_seabed_depth(lon, lat)
            current_depth = self.compute_rope_length(self.pulley_turns, self.pulley_diameter, self.half_diagonal)
            self.log_data(lon, lat, seabed, seabed + self.target_offset if seabed else 0, current_depth)

            self.update_gui({
                "Time": now_time,
                "Longitude": f"{lon:.5f}",
                "Latitude": f"{lat:.5f}",
                "Max Depth": f"{seabed:.2f}" if seabed else "--",
                "Current Depth": f"{current_depth:.2f}"
            })

            self.time_history.append(now_time)
            self.max_depth_history.append(seabed)
            self.cur_depth_history.append(current_depth)
            if len(self.time_history) > 20:
                self.time_history.pop(0)
                self.max_depth_history.pop(0)
                self.cur_depth_history.pop(0)

            self.update_plot()
            time.sleep(1)

    def on_close(self):
        self.excel_workbook.close()
        GPIO.cleanup()
        self.root.destroy()

if __name__ == "__main__":
    DepthControlSystem()
