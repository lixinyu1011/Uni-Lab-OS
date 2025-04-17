import tkinter as tk
from tkinter import ttk  # 使用 ttk 替换 tk 控件
from tkinter import messagebox
from tkinter.font import Font
from threading import Thread
from ttkthemes import ThemedTk
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import clr  # pythonnet library
import sys
import threading
import datetime

jifenshijian = 10 #积分时间
shuaxinshijian = 0.01 #刷新时间
zihaodaxiao = 16 #字号大小
ymax = 70000
ymin = -2000

# 加载DLL
dll_path = "C:\\auto\\UV_spec\\idea-sdk 3.0.9\\idea-sdk.UPI\\IdeaOptics.dll"
clr.AddReference(dll_path)
from IdeaOptics import Wrapper

# 初始化Wrapper对象和光谱仪
wrapper = Wrapper()
number_of_spectrometers = wrapper.OpenAllSpectrometers()
if number_of_spectrometers > 0:
    spectrometer_index = 0  # 假设使用第一个光谱仪
    integration_time = jifenshijian  # 设置积分时间
    wrapper.setIntegrationTime(spectrometer_index, integration_time)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("光谱测试")
        self.is_continuous = False
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.stop_event = threading.Event()  # 使用Event代替布尔标志
        self.continuous_thread = None  # 在这里初始化
        self.background_spectrum = None
        self.correct_background = False
        self.test_count = 0
        self.background_count = 0

        self.source_spectrum = None  # 初始化光源强度变量
        self.transmission_mode = False  # 初始化透射模式标志

        self.data_ready = False

        # 使用 grid 布局
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        self.root.rowconfigure(3, weight=1)

        self.current_ylim = [-100, 1000]  # 初始化y轴范围

        # 创建一个 Style 对象
        style = ttk.Style()

        # 定义一个新的样式
        style.configure('Custom.TButton', font=('Helvetica', zihaodaxiao, 'bold'), foreground='white')

        # 创建滑动条和按钮的容器 Frame
        control_frame = ttk.Frame(self.root)
        control_frame.grid(row=0, column=0, sticky="ew")

        # 创建一个滑动条来选择平滑次数
        self.boxcar_width_slider = tk.Scale(control_frame, from_=0, to=10, orient=tk.HORIZONTAL, length=300, label="平滑次数", font=("Helvetica", zihaodaxiao, 'bold'))
        self.boxcar_width_slider.grid(row=0, column=0, padx=10, pady=10)

        # 创建一个滑动条来选择平均次数
        self.scans_to_average_slider = tk.Scale(control_frame, from_=1, to=10, orient=tk.HORIZONTAL, length=300, label="平均次数", font=("Helvetica", zihaodaxiao, 'bold'))
        self.scans_to_average_slider.grid(row=0, column=1, padx=10, pady=10)

        # 调整 Scale 控件的外观
        self.boxcar_width_slider.config(bg='grey', fg='white')
        self.scans_to_average_slider.config(bg='grey', fg='white')

        # 字体设置
        entry_font = ('Helvetica', zihaodaxiao, 'bold')

        # 添加输入框的容器 Frame
        entry_frame = ttk.Frame(self.root)
        entry_frame.grid(row=1, column=0, sticky="ew")

        # 创建并放置"积分时间(ms)"输入框
        ttk.Label(entry_frame, text="积分时间(ms):", font=entry_font).grid(row=0, column=0, padx=10, pady=10)
        self.integration_time_entry = ttk.Entry(entry_frame, font=entry_font)
        self.integration_time_entry.grid(row=0, column=1, padx=10, pady=10)
        self.integration_time_entry.insert(0, "10")  # 设置默认值

        # 创建并放置"刷新间隔(s)"输入框
        ttk.Label(entry_frame, text="刷新间隔(s):", font=entry_font).grid(row=0, column=2, padx=10, pady=10)
        self.refresh_interval_entry = ttk.Entry(entry_frame, font=entry_font)
        self.refresh_interval_entry.grid(row=0, column=3, padx=10, pady=10)
        self.refresh_interval_entry.insert(0, "0.01")  # 设置默认值

        # 创建按钮的容器 Frame
        button_frame = ttk.Frame(self.root)
        button_frame.grid(row=2, column=0, sticky="ew")

        # 创建并放置按钮
        ttk.Button(button_frame, text="测试一下", style='Custom.TButton', command=self.single_test).grid(row=0, column=0, padx=10, pady=10)
        ttk.Button(button_frame, text="连续测试", style='Custom.TButton', command=self.start_continuous_test).grid(row=0, column=1, padx=10, pady=10)
        ttk.Button(button_frame, text="停止测试", style='Custom.TButton', command=self.stop_continuous_test).grid(row=0, column=2, padx=10, pady=10)

        # 创建背景相关按钮的容器 Frame
        background_frame = ttk.Frame(self.root)
        background_frame.grid(row=3, column=0, sticky="ew")

        # 创建并放置“采集背景”按钮
        self.collect_background_button = ttk.Button(background_frame, text="采集背景", style='Custom.TButton', command=self.collect_background)
        self.collect_background_button.grid(row=0, column=0, padx=10, pady=10)

        # 创建并放置“背景校正”按钮
        self.correct_background_button = ttk.Button(background_frame, text="背景校正", style='Custom.TButton', command=self.toggle_background_correction)
        self.correct_background_button.grid(row=0, column=1, padx=10, pady=10)

        # 创建“光源采集”按钮
        ttk.Button(background_frame, text="光源采集", style='Custom.TButton', command=self.collect_source).grid(row=0, column=2, padx=10, pady=10)

        # 创建“透射模式”按钮
        self.transmission_button = ttk.Button(background_frame, text="透射模式", style='Custom.TButton', command=self.toggle_transmission_mode)
        self.transmission_button.grid(row=0, column=3, padx=10, pady=10)

        # 创建 matplotlib 画布
        plt.style.use('ggplot')  # 使用预定义的样式，如 'ggplot'
        self.figure, self.ax = plt.subplots(figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.figure, self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=3, column=0, sticky="ew")
        
        # 使用 grid 布局来放置 matplotlib 画布
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=4, column=0, sticky="ew")
        
        # 创建文件名并打开文件
        start_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_file = open(f"C:\\auto\\UV_spec\\data\\{start_time}.txt", "w")
    
    def get_spectrum_data(self):
        # 获取波长和光谱值
        pixels = wrapper.getNumberOfPixels(spectrometer_index)
        spectrum = wrapper.getSpectrum(spectrometer_index)
        wavelengths = wrapper.getWavelengths(spectrometer_index)
        
        # 转换.NET数组到Python列表
        spectrum_list = [spectrum[i] for i in range(pixels)]
        wavelengths_list = [wavelengths[i] for i in range(pixels)]

        self.data_ready = True
        return wavelengths_list, spectrum_list

    def collect_source(self):
        # 采集光源强度数据
        wavelengths, self.source_spectrum = self.get_spectrum_data()
        conditions = f"jifenshijian = {jifenshijian}  shuaxinshijian = {shuaxinshijian}  zihaodaxiao = {zihaodaxiao}"
        self.write_data_to_file("source", 1, conditions, self.source_spectrum)
        self.update_plot(wavelengths, self.source_spectrum)

    def toggle_transmission_mode(self):
        # 切换透射模式
        self.transmission_mode = not self.transmission_mode
        self.transmission_button.config(text="正在透射" if self.transmission_mode else "透射模式")

    def calculate_transmission(self, spectrum):
        # 计算透射率
        transmission = []
        for s, b, src in zip(spectrum, self.background_spectrum, self.source_spectrum):
            denominator = max(src - b, 0.1)
            trans_value = (s - b) / denominator * 100
            trans_value = max(0, min(trans_value, 100))
            transmission.append(trans_value)
        return transmission

    def update_plot(self, wavelengths, spectrum, plot_type='spectrum'):

        if not self.data_ready:
            return
        
        self.ax.clear()

        if plot_type == 'transmission':
            # 透射率模式的绘图设置
            self.ax.plot(wavelengths, spectrum, label='Transmission (%)')
            self.ax.set_ylim(-10, 110)  # 设置y轴范围为0%到100%
            self.ax.set_ylabel('Transmission (%)', fontname='Arial', fontsize=zihaodaxiao)
        else:
            # 普通光谱模式的绘图设置
            self.ax.plot(wavelengths, spectrum)
            self.ax.set_ylim(self.current_ylim)  # 使用当前y轴范围

            # 计算新的最大值和最小值
            new_min, new_max = min(spectrum), max(spectrum)

            # 检查新的最大值或最小值是否超过当前y轴范围
            while new_min < self.current_ylim[0] or new_max > self.current_ylim[1]:
                # 扩大y轴范围
                self.current_ylim = [self.current_ylim[0] * 2, self.current_ylim[1] * 2]

                # 确保新的y轴范围不超过最大限制
                if self.current_ylim[0] < ymin:
                    self.current_ylim[0] = ymin
                if self.current_ylim[1] > ymax:
                    self.current_ylim[1] = ymax
                break

            self.ax.set_ylabel('Intensity', fontname='Arial', fontsize=zihaodaxiao)

        self.ax.set_xlabel('Wavelength (nm)', fontname='Arial', fontsize=zihaodaxiao)
        self.ax.set_title('Spectrum', fontname='Arial', fontsize=zihaodaxiao)

        self.canvas.draw()

        self.data_ready = False
        
    def draw_plot(self):
        self.canvas.draw()

    def write_data_to_file(self, test_type, test_number, conditions, spectrum):
        data_str = " ".join(map(str, spectrum))
        self.data_file.write(f"{test_type}{test_number}\n{conditions}\n{data_str}\n\n")
        self.data_file.flush()

    def collect_background(self):
        # 设置平滑次数
        boxcar_width = self.boxcar_width_slider.get()
        wrapper.setBoxcarWidth(spectrometer_index, boxcar_width)

        # 设置平均次数
        scans_to_average = self.scans_to_average_slider.get()
        wrapper.setScansToAverage(spectrometer_index, scans_to_average)

        # 采集背景数据
        wavelengths, self.background_spectrum = self.get_spectrum_data()
        conditions = f"jifenshijian = {jifenshijian}  shuaxinshijian = {shuaxinshijian}  zihaodaxiao = {zihaodaxiao}  pinghuacishu = {self.boxcar_width_slider.get()}  pingjuncishu = {self.scans_to_average_slider.get()}"
        self.background_count += 1
        self.write_data_to_file("background", self.background_count, conditions, self.background_spectrum)
        self.update_plot(wavelengths, self.background_spectrum)

    def toggle_background_correction(self):
        self.correct_background = not self.correct_background
        self.correct_background_button.config(text="正在校正" if self.correct_background else "背景校正")
    
    def apply_background_correction(self, spectrum):
        if self.background_spectrum is not None and self.correct_background:
            return [s - b for s, b in zip(spectrum, self.background_spectrum)]
        return spectrum
    
    def single_test(self):
        # 获取输入框的值
        jifenshijian = float(self.integration_time_entry.get())
        shuaxinshijian = float(self.refresh_interval_entry.get())

        # 设置平滑次数
        boxcar_width = self.boxcar_width_slider.get()
        wrapper.setBoxcarWidth(spectrometer_index, boxcar_width)

        # 设置平均次数
        scans_to_average = self.scans_to_average_slider.get()
        wrapper.setScansToAverage(spectrometer_index, scans_to_average)

        conditions = f"jifenshijian = {jifenshijian}  shuaxinshijian = {shuaxinshijian}  zihaodaxiao = {zihaodaxiao}  pinghuacishu = {self.boxcar_width_slider.get()}  pingjuncishu = {self.scans_to_average_slider.get()}"
        self.test_count += 1
        
        wavelengths, spectrum = self.get_spectrum_data()

        # 在透射模式下计算透射率，否则应用背景校正
        if self.transmission_mode and self.background_spectrum is not None and self.source_spectrum is not None:
            transmission = self.calculate_transmission(spectrum)
            self.update_plot(wavelengths, transmission, plot_type='transmission')
        else:
            corrected_spectrum = self.apply_background_correction(spectrum)
            self.update_plot(wavelengths, corrected_spectrum, plot_type='spectrum')
        
    def continuous_test(self):
        while not self.stop_event.is_set():
            # 获取输入框的值
            jifenshijian = float(self.integration_time_entry.get())
            shuaxinshijian = float(self.refresh_interval_entry.get())
            
            # 设置平滑次数和平均次数
            boxcar_width = self.boxcar_width_slider.get()
            wrapper.setBoxcarWidth(spectrometer_index, boxcar_width)
            scans_to_average = self.scans_to_average_slider.get()
            wrapper.setScansToAverage(spectrometer_index, scans_to_average)

            conditions = f"jifenshijian = {jifenshijian}  shuaxinshijian = {shuaxinshijian}  zihaodaxiao = {zihaodaxiao}  pinghuacishu = {self.boxcar_width_slider.get()}  pingjuncishu = {self.scans_to_average_slider.get()}"
            self.test_count += 1
            wavelengths, spectrum = self.get_spectrum_data()
            self.write_data_to_file("test", self.test_count, conditions, spectrum)

            # 根据当前模式计算并更新图表
            if self.transmission_mode and self.background_spectrum is not None and self.source_spectrum is not None:
                transmission = self.calculate_transmission(spectrum)
                self.update_plot(wavelengths, transmission, plot_type='transmission')
            else:
                corrected_spectrum = self.apply_background_correction(spectrum)
                self.update_plot(wavelengths, corrected_spectrum)

            time.sleep(shuaxinshijian)

    def start_continuous_test(self):
        self.stop_event.clear()  # 重置事件
        self.continuous_thread = Thread(target=self.continuous_test)
        self.continuous_thread.start()

    def stop_continuous_test(self):
        self.stop_event.set()  # 设置事件通知线程停止
        self.continuous_thread = None

    def on_closing(self):
        if self.data_file:
            self.data_file.close()
        if messagebox.askyesno("退出", "实验g了？"):
            self.stop_continuous_test()
            self.root.destroy()
            sys.exit()

if __name__ == "__main__":
    # 使用 ThemedTk 而不是普通的 Tk
    root = ThemedTk(theme="equilux")  # 使用 'arc' 主题

    # 由于我们已经使用了 ttkthemes 来设置主题，下面这些行可以省略
    # style = ttk.Style()
    # style.theme_use('arc')

    app = App(root)
    root.mainloop()
