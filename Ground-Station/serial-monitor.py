import serial.tools.list_ports
import datetime
import csv
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import sys
import threading  # <-- Added for multi-threading


class GroundStationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ground Station Data Viewer")
        self.root.geometry("1200x600")  # Adjusted width to fit graphs and settings
        self.root.configure(bg="#f0f0f0")

        # Set window icon to image at icon_path
        icon_path = "C:/Users/noppa/Desktop/Ground-Station/logo.png"
        try:
            self.root.iconphoto(True, tk.PhotoImage(file=icon_path))
        except Exception:
            pass

        # Create a canvas and a scrollable frame
        self.scroll_canvas = tk.Canvas(self.root, bg="#f0f0f0")
        self.scrollbar = ttk.Scrollbar(self.root, orient=tk.VERTICAL, command=self.scroll_canvas.yview)
        self.scrollable_frame = ttk.Frame(self.scroll_canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))
        )
        self.scroll_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.scroll_canvas.configure(yscrollcommand=self.scrollbar.set)

        self.scroll_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Serial and recording variables
        self.serial_inst = serial.Serial(timeout=0.1, 
                                       write_timeout=0.1,
                                       xonxoff=False,     # ปิดการควบคุม flow
                                       rtscts=False,      # ปิดการควบคุม flow
                                       dsrdtr=False)      # ปิดการควบคุม flow
        self.serial_inst2 = serial.Serial(timeout=0.1,
                                        write_timeout=0.1,
                                        xonxoff=False,
                                        rtscts=False, 
                                        dsrdtr=False)
        self.recording = False
        self.file_path = "received_data.csv"
        self.connect_time = None
        self.disconnect_time = None
        self.x_data = []
        self.y_data = []
        self.x_data2 = []
        self.y_data2 = []
        self.start_time = None
        self.start_time2 = None  # Separate start time for Data2

        # Threading variables
        self.com1_thread = None  # Thread for COM1
        self.com2_thread = None  # Thread for COM2
        self.com1_thread_stop = threading.Event()
        self.com2_thread_stop = threading.Event()
        self.reading_serial1 = False
        self.reading_serial2 = False

        # Initialize GUI components inside the scrollable frame
        self.create_widgets()

    def create_widgets(self):
        # Logo Picture (kept on the far left)
        icon_path = "C:/Users/noppa/Desktop/Ground-Station/logo.png"
        self.logo_image = tk.PhotoImage(file=icon_path)
        self.logo_label = tk.Label(self.scrollable_frame, image=self.logo_image, bg="#f0f0f0")
        self.logo_label.grid(row=0, column=0, rowspan=3, padx=10, pady=5, sticky=tk.N)

        # Title Label
        title_label = ttk.Label(self.scrollable_frame, text="Ground Station Data Viewer", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=1, columnspan=2, padx=10, pady=10)

        # Status Monitor
        self.status_frame = ttk.LabelFrame(self.scrollable_frame, text="Status Monitor", padding="10")
        self.status_frame.grid(row=1, column=1, padx=10, pady=10, sticky=(tk.W, tk.E))

        # COM Port Status Label (bold font)
        self.com_status_label = ttk.Label(
            self.status_frame,
            text="COM Port Status: Disconnected",
            font=("Arial", 12, "bold"),
            foreground="red"
        )
        self.com_status_label.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Online Time Label
        self.time_info_label = ttk.Label(self.status_frame, text="Online Time: N/A", font=("Arial", 12))
        self.time_info_label.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)

        # Combined Actions and Connection Settings
        self.control_frame = ttk.LabelFrame(self.scrollable_frame, text="Controls", padding="10")
        self.control_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))

        # Connection Settings
        ttk.Label(self.control_frame, text="Select COM Port1:").grid(row=0, column=0, padx=5, pady=5)
        self.port_combobox = ttk.Combobox(self.control_frame, values=self.get_ports(), state="readonly", width=10)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(self.control_frame, text="Select Baud Rate:").grid(row=1, column=0, padx=5, pady=5)
        self.baudrate_combobox = ttk.Combobox(self.control_frame, values=["9600", "19200", "38400", "57600", "115200"], state="readonly", width=10)
        self.baudrate_combobox.grid(row=1, column=1, padx=5, pady=5)
        self.baudrate_combobox.set(" ")

        # Action Buttons for Port 1
        self.connect_btn = tk.Button(self.control_frame, text="Connect Port1", command=self.start_serial)
        self.connect_btn.grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Button(self.control_frame, text="Disconnect Port", command=self.disconnect_serial).grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)

        # --- Serial Port 2 controls below Port 1 ---
        ttk.Label(self.control_frame, text="Select COM Port2:").grid(row=3, column=0, padx=5, pady=5)
        self.port2_combobox = ttk.Combobox(self.control_frame, values=self.get_ports(), state="readonly", width=10)
        self.port2_combobox.grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(self.control_frame, text="Select Baud Rate:").grid(row=4, column=0, padx=5, pady=5)
        self.baudrate2_combobox = ttk.Combobox(self.control_frame, values=["9600", "19200", "38400", "57600", "115200"], state="readonly", width=10)
        self.baudrate2_combobox.grid(row=4, column=1, padx=5, pady=5)
        self.baudrate2_combobox.set(" ")

        self.connect2_btn = tk.Button(self.control_frame, text="Connect Port2", command=self.start_serial2)
        self.connect2_btn.grid(row=5, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Button(self.control_frame, text="Disconnect Port", command=self.disconnect_serial2).grid(row=5, column=1, padx=5, pady=5, sticky=tk.W)

        self.record_btn = tk.Button(self.control_frame, text="Record Data", command=self.start_recording)
        self.record_btn.grid(row=6, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Button(self.control_frame, text="Stop Record", command=self.stop_recording).grid(row=6, column=1, padx=5, pady=5, sticky=tk.W)

        ttk.Button(self.control_frame, text="Change File Location", command=self.change_file_location).grid(row=7, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Button(self.control_frame, text="Clear Interface", command=self.clear_display).grid(row=7, column=1, padx=5, pady=5, sticky=tk.W)

        # Add time unit selection dropdown
        self.time_unit = tk.StringVar(value="Seconds")
        ttk.Label(self.control_frame, text="Time Unit:").grid(row=8, column=0, padx=5, pady=5, sticky=tk.W)
        self.time_unit_combobox = ttk.Combobox(
            self.control_frame, 
            textvariable=self.time_unit, 
            values=["Seconds", "Milliseconds", "Minutes"], 
            state="readonly", 
            width=12
        )
        self.time_unit_combobox.grid(row=8, column=1, padx=5, pady=5)

        # Add Reset All button
        ttk.Button(self.control_frame, text="Reset All", command=self.reset_all).grid(row=9, column=0, columnspan=2, padx=5, pady=5, sticky=tk.EW)

        # Raw Data Text Display for Real-Time Graph
        self.raw_data_frame = ttk.LabelFrame(self.scrollable_frame, text="Raw Data (Real-Time Graph)", padding="10")
        self.raw_data_frame.grid(row=3, column=1, padx=10, pady=10, sticky=(tk.W, tk.E))

        # Add scrollbar to the text display
        self.raw_data_scrollbar = ttk.Scrollbar(self.raw_data_frame, orient=tk.VERTICAL)
        self.display_text = tk.Text(
            self.raw_data_frame, 
            height=10, 
            width=50, 
            state="normal", 
            bg="#ffffff", 
            fg="#333", 
            font=("Courier", 10), 
            yscrollcommand=self.raw_data_scrollbar.set
        )
        self.raw_data_scrollbar.config(command=self.display_text.yview)
        self.raw_data_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.display_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Raw Data Text Display for Data2
        self.raw_data_frame2 = ttk.LabelFrame(self.scrollable_frame, text="Raw Data (Data2)", padding="10")
        self.raw_data_frame2.grid(row=3, column=2, padx=10, pady=10, sticky=(tk.W, tk.E))

        # Add scrollbar to the text display for Data2
        self.raw_data_scrollbar2 = ttk.Scrollbar(self.raw_data_frame2, orient=tk.VERTICAL)
        self.display_text2 = tk.Text(
            self.raw_data_frame2, 
            height=10, 
            width=50, 
            state="normal", 
            bg="#ffffff", 
            fg="#333", 
            font=("Courier", 10), 
            yscrollcommand=self.raw_data_scrollbar2.set
        )
        self.raw_data_scrollbar2.config(command=self.display_text2.yview)
        self.raw_data_scrollbar2.pack(side=tk.RIGHT, fill=tk.Y)
        self.display_text2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Real-Time Graph
        self.graph_frame = ttk.LabelFrame(self.scrollable_frame, text="Real-Time Graph", padding="10")
        self.graph_frame.grid(row=2, column=1, padx=10, pady=10, sticky=(tk.W, tk.E))

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title("Real-Time Data", fontsize=14, fontweight="bold", color="black")
        self.ax.set_xlabel("Time (s)", fontsize=12, fontweight="bold", color="black")
        self.ax.set_ylabel("Value", fontsize=12, fontweight="bold", color="black")
        self.ax.grid(True, which="major", linestyle="-", linewidth=0.75, color="lightgray")
        self.ax.grid(True, which="minor", linestyle=":", linewidth=0.5, color="lightgray")
        self.ax.minorticks_on()
        self.ax.set_facecolor("white")
        self.line, = self.ax.plot([], [], lw=2, color="blue", label="Data")
        self.ax.legend(loc="upper left", fontsize=10, frameon=True, edgecolor="black")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack()

        # Add horizontal scrollbar for x-axis adjustment
        self.x_scrollbar = ttk.Scale(
            self.graph_frame, 
            from_=0, 
            to=100, 
            orient=tk.HORIZONTAL, 
            command=self.adjust_x_axis
        )
        self.x_scrollbar.pack(fill=tk.X, padx=5, pady=5)

        graph_buttons_frame = ttk.Frame(self.graph_frame)
        graph_buttons_frame.pack(pady=5)
        ttk.Button(graph_buttons_frame, text="Clear Graph", command=self.clear_graph).grid(row=0, column=0, padx=5)
        ttk.Button(graph_buttons_frame, text="Save Graph", command=self.save_graph).grid(row=0, column=1, padx=5)

        # Second Real-Time Graph
        self.graph_frame2 = ttk.LabelFrame(self.scrollable_frame, text="Data2", padding="10")
        self.graph_frame2.grid(row=2, column=2, padx=10, pady=10, sticky=(tk.W, tk.E))

        self.fig2, self.ax2 = plt.subplots(figsize=(6, 4))
        self.ax2.set_title("Data2 Graph", fontsize=14, fontweight="bold", color="black")
        self.ax2.set_xlabel("Time (s)", fontsize=12, fontweight="bold", color="black")
        self.ax2.set_ylabel("Value", fontsize=12, fontweight="bold", color="black")
        self.ax2.grid(True, which="major", linestyle="-", linewidth=0.75, color="lightgray")
        self.ax2.grid(True, which="minor", linestyle=":", linewidth=0.5, color="lightgray")
        self.ax2.minorticks_on()
        self.ax2.set_facecolor("white")
        self.line2, = self.ax2.plot([], [], lw=2, color="red", label="Data2")
        self.ax2.legend(loc="upper left", fontsize=10, frameon=True, edgecolor="black")

        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.graph_frame2)
        self.canvas_widget2 = self.canvas2.get_tk_widget()
        self.canvas_widget2.pack()

        # Add horizontal scrollbar for x-axis adjustment
        self.x_scrollbar2 = ttk.Scale(
            self.graph_frame2, 
            from_=0, 
            to=100, 
            orient=tk.HORIZONTAL, 
            command=self.adjust_x_axis2
        )
        self.x_scrollbar2.pack(fill=tk.X, padx=5, pady=5)

        graph_buttons_frame2 = ttk.Frame(self.graph_frame2)
        graph_buttons_frame2.pack(pady=5)
        ttk.Button(graph_buttons_frame2, text="Clear Graph", command=self.clear_graph2).grid(row=0, column=0, padx=5)
        ttk.Button(graph_buttons_frame2, text="Save Graph", command=self.save_graph2).grid(row=0, column=1, padx=5)

        # Handle program exit
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def start_serial(self):
        selected_port = self.port_combobox.get()
        selected_baudrate = self.baudrate_combobox.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Please select a COM port.")
            return
        if not selected_baudrate or not selected_baudrate.isdigit():
            messagebox.showerror("Error", "Please select a valid Baud Rate.")
            return
        try:
            self.serial_inst.port = selected_port
            self.serial_inst.baudrate = int(selected_baudrate)
            self.serial_inst.timeout = 0.1
            self.serial_inst.open()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open port: {e}")
            return
        self.connect_time = datetime.datetime.now()
        self.disconnect_time = None
        self.update_com_status_label()
        self.connect_btn.config(bg="#28a745", fg="white", activebackground="#218838", activeforeground="white", font=("Arial", 10, "bold"))
        # Start non-blocking serial read
        if not self.reading_serial1:
            self.reading_serial1 = True
            self.root.after(10, self.read_serial1_loop)

    def disconnect_serial(self):
        if self.serial_inst.is_open:
            self.serial_inst.close()
            self.disconnect_time = datetime.datetime.now()
            self.update_com_status_label()
            self.connect_btn.config(bg="SystemButtonFace", fg="black", activebackground="SystemButtonFace", activeforeground="black", font=("Arial", 10, "normal"))
        self.reading_serial1 = False

    def start_serial2(self):
        selected_port = self.port2_combobox.get()
        selected_baudrate = self.baudrate2_combobox.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Please select a COM2 port.")
            return
        if not selected_baudrate or not selected_baudrate.isdigit():
            messagebox.showerror("Error", "Please select a valid Baud Rate for COM2.")
            return
        try:
            self.serial_inst2.port = selected_port
            self.serial_inst2.baudrate = int(selected_baudrate)
            self.serial_inst2.timeout = 0.1
            self.serial_inst2.open()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open COM2: {e}")
            return
        self.update_com_status_label()
        self.connect2_btn.config(bg="#28a745", fg="white", activebackground="#218838", activeforeground="white", font=("Arial", 10, "bold"))
        # Start non-blocking serial read
        if not self.reading_serial2:
            self.reading_serial2 = True
            self.root.after(10, self.read_serial2_loop)

    def disconnect_serial2(self):
        if self.serial_inst2.is_open:
            self.serial_inst2.close()
            self.update_com_status_label()
            self.connect2_btn.config(bg="SystemButtonFace", fg="black", activebackground="SystemButtonFace", activeforeground="black", font=("Arial", 10, "normal"))
        self.reading_serial2 = False

    def update_com_status_label(self):
        status = []
        if self.serial_inst.is_open:
            status.append(f"COM1: {self.serial_inst.port} (Connected)")
        else:
            status.append("COM1: Disconnected")
        if self.serial_inst2.is_open:
            status.append(f"COM2: {self.serial_inst2.port} (Connected)")
        else:
            status.append("COM2: Disconnected")
        color = "green" if self.serial_inst.is_open or self.serial_inst2.is_open else "red"
        self.com_status_label.config(text=" | ".join(status), foreground=color)

    def change_file_location(self):
        self.file_path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if self.file_path:
            messagebox.showinfo("File Location Changed", f"Data will be saved to: {self.file_path}")
        else:
            self.file_path = "received_data.csv"

    def start_recording(self):
        if not self.serial_inst.is_open and not self.serial_inst2.is_open:
            messagebox.showerror("Error", "Please connect to at least one COM port first.")
            return
        # Open separate files for each port
        self.file1 = None
        self.file2 = None
        self.writer1 = None
        self.writer2 = None
        try:
            if self.serial_inst.is_open:
                self.file1 = open("received_data_com1.csv", mode="w", newline="")
                self.writer1 = csv.writer(self.file1)
                self.writer1.writerow(["Timestamp1", "Data1"])
            if self.serial_inst2.is_open:
                self.file2 = open("received_data_com2.csv", mode="w", newline="")
                self.writer2 = csv.writer(self.file2)
                self.writer2.writerow(["Timestamp2", "Data2"])
        except PermissionError as e:
            messagebox.showerror("Permission Error", f"Cannot write to file:\n{e.filename}\n\nPlease close the file if it is open in another program and try again.")
            # Close any opened files
            if self.file1: self.file1.close()
            if self.file2: self.file2.close()
            return
        except Exception as e:
            messagebox.showerror("File Error", f"Failed to open file:\n{e}")
            if self.file1: self.file1.close()
            if self.file2: self.file2.close()
            return
        self.recording = True
        self.com1_thread_stop.clear()
        self.com2_thread_stop.clear()
        self.record_btn.config(bg="#dc3545", fg="white", activebackground="#c82333", activeforeground="white", font=("Arial", 10, "bold"))
        # Start threads for each port
        if self.serial_inst.is_open:
            self.com1_thread = threading.Thread(target=self.read_data_com1, daemon=True)
            self.com1_thread.start()
        if self.serial_inst2.is_open:
            self.com2_thread = threading.Thread(target=self.read_data_com2, daemon=True)
            self.com2_thread.start()

    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.com1_thread_stop.set()
            self.com2_thread_stop.set()
            # Reset Record Data button style
            self.record_btn.config(bg="SystemButtonFace", fg="black", activebackground="SystemButtonFace", activeforeground="black", font=("Arial", 10, "normal"))
            stop_time = datetime.datetime.now()
            try:
                if hasattr(self, "file1") and self.file1:
                    self.file1.close()
                if hasattr(self, "file2") and self.file2:
                    self.file2.close()
                # ...existing code for UI updates...
            except AttributeError:
                pass
            messagebox.showinfo("Stopped", f"Stopped recording data at {stop_time.strftime('%H:%M:%S')}")
        else:
            messagebox.showwarning("Warning", "Recording is not active.")

    def read_serial1_loop(self):
        if self.serial_inst.is_open:
            try:
                if self.serial_inst.in_waiting:
                    packet = self.serial_inst.readline()
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                    try:
                        # decode แบบ latin-1 เพื่อไม่ให้ error และเก็บทุก byte
                        data = packet.decode('latin-1').strip()
                        # ถ้าเจอข้อมูลที่ขึ้นต้นด้วย [915 MHz] ให้บันทึกใน CSV แบบมีเครื่องหมาย quote
                        if data.startswith("[915 MHz]"):
                            # escape double quote ในข้อมูล
                            data_escaped = data.replace('"', '""')
                            csv_data = f'"{data_escaped}"'
                            self.display_text.insert(tk.END, f"{timestamp},{csv_data}\n")
                            if self.recording and hasattr(self, "writer1") and self.writer1:
                                self.writer1.writerow([timestamp, data])
                            # Try to extract a float from the data and plot
                            try:
                                value = float(data.split()[-1])
                                self.update_graph(value)
                            except Exception:
                                pass
                        else:
                            # ข้อมูลอื่น (noise หรือ binary) ให้บันทึกแบบ raw ไม่ใส่ quote
                            self.display_text.insert(tk.END, f"{timestamp},{data}\n")
                            if self.recording and hasattr(self, "writer1") and self.writer1:
                                self.writer1.writerow([timestamp, data])
                            # Try to plot if data is a number
                            try:
                                value = float(data)
                                self.update_graph(value)
                            except Exception:
                                pass
                        self.display_text.see(tk.END)
                    except Exception:
                        # ถ้า decode ไม่ได้จริงๆ ให้บันทึกเป็น hex
                        data = packet.hex()
                        self.display_text.insert(tk.END, f"{timestamp},RAW:{data}\n")
                        if self.recording and hasattr(self, "writer1") and self.writer1:
                            self.writer1.writerow([timestamp, f"RAW:{data}"])
            except serial.SerialException:
                self.disconnect_serial()
                return

        if self.reading_serial1:
            self.root.after(1, self.read_serial1_loop)

    def read_serial2_loop(self):
        if self.serial_inst2.is_open:
            try:
                if self.serial_inst2.in_waiting:
                    packet2 = self.serial_inst2.readline()
                    timestamp2 = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    try:
                        data2 = packet2.decode('utf-8', errors='ignore').strip()
                        
                        if data2:
                            self.display_text2.insert(tk.END, f"{timestamp2} - {data2}\n")
                            self.display_text2.see(tk.END)
                            if self.recording and hasattr(self, "writer2") and self.writer2:
                                self.writer2.writerow([timestamp2, data2])
                            # Try to plot if data2 is a number
                            try:
                                value2 = float(data2)
                                self.update_graph2(value2)
                            except Exception:
                                pass
                    except UnicodeDecodeError:
                        pass
                        
            except serial.SerialException:
                self.disconnect_serial2()
                return
                
        if self.reading_serial2:
            self.root.after(1, self.read_serial2_loop)

    def update_graph(self, new_value):
        if self.start_time is None:
            self.start_time = datetime.datetime.now()
        elapsed_time = (datetime.datetime.now() - self.start_time).total_seconds()
        # Adjust time based on the selected unit
        if self.time_unit.get() == "Milliseconds":
            elapsed_time *= 1000
        elif self.time_unit.get() == "Minutes":
            elapsed_time /= 60
        self.x_data.append(elapsed_time)
        self.y_data.append(new_value)
        if self.x_data:
            self.line.set_data(self.x_data, self.y_data)
            self.ax.set_xlim(0, max(self.x_data))
            self.ax.set_ylim(-3, 3)  # Set y-axis from -3 to 3
        else:
            self.ax.set_xlim(0, 1)
            self.ax.set_ylim(-3, 3)
        # Update x-axis label dynamically
        self.ax.set_xlabel(f"Time ({self.time_unit.get()})", fontsize=12, fontweight="bold", color="black")
        self.line.set_linewidth(0.5)  # Reduced line thickness
        self.canvas.draw()

    def update_graph2(self, new_value):
        if self.start_time2 is None:
            self.start_time2 = datetime.datetime.now()
        elapsed_time = (datetime.datetime.now() - self.start_time2).total_seconds()
        # Adjust time based on the selected unit
        if self.time_unit.get() == "Milliseconds":
            elapsed_time *= 1000
        elif self.time_unit.get() == "Minutes":
            elapsed_time /= 60
        self.x_data2.append(elapsed_time)
        self.y_data2.append(new_value)
        # Clear the previous scatter points and plot new ones
        self.ax2.clear()
        self.ax2.set_title("Data2 Graph", fontsize=14, fontweight="bold", color="black")
        self.ax2.set_xlabel(f"Time ({self.time_unit.get()})", fontsize=12, fontweight="bold", color="black")
        self.ax2.set_ylabel("Value", fontsize=12, fontweight="bold", color="black")
        self.ax2.grid(True, which="major", linestyle="-", linewidth=0.75, color="lightgray")
        self.ax2.grid(True, which="minor", linestyle=":", linewidth=0.5, color="lightgray")
        self.ax2.minorticks_on()
        self.ax2.set_facecolor("white")
        # Plot the data as scatter points with reduced size
        self.ax2.scatter(self.x_data2, self.y_data2, color="red", s=5, label="Data2")
        self.ax2.legend(loc="upper left", fontsize=10, frameon=True, edgecolor="black")
        # Adjust the axes limits
        self.ax2.set_xlim(0, max(self.x_data2) + 1)
        self.ax2.set_ylim(-3, 3)  # Set y-axis from -3 to 3
        # Redraw the canvas
        self.canvas2.draw()

    def adjust_x_axis(self, value):
        """Adjust the x-axis range manually using the scrollbar."""
        if self.x_data:
            max_x = max(self.x_data)
            new_limit = max_x * (float(value) / 100)
            self.ax.set_xlim(0, new_limit)
            self.canvas.draw()

    def adjust_x_axis2(self, value):
        """Adjust the x-axis range manually for the Data2 graph using the scrollbar."""
        if self.x_data2:
            max_x = max(self.x_data2)
            new_limit = max_x * (float(value) / 100)
            self.ax2.set_xlim(0, new_limit)
            self.canvas2.draw()

    def clear_graph(self):
        self.x_data.clear()
        self.y_data.clear()
        self.line.set_data([], [])
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(-3, 3)
        self.canvas.draw()

    def clear_graph2(self):
        self.x_data2.clear()
        self.y_data2.clear()
        self.line2.set_data([], [])
        self.ax2.set_xlim(0, 1)
        self.ax2.set_ylim(-3, 3)
        self.canvas2.draw()

    def save_graph(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png"), ("All files", "*.*")])
        if file_path:
            self.fig.savefig(file_path)
            messagebox.showinfo("Graph Saved", f"Graph saved to {file_path}")

    def save_graph2(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png"), ("All files", "*.*")])
        if file_path:
            self.fig2.savefig(file_path)
            messagebox.showinfo("Graph Saved", f"Graph saved to {file_path}")

    def clear_display(self):
        self.display_text.delete("1.0", tk.END)
        self.display_text2.delete("1.0", tk.END)

    def change_time_unit(self, ax, graph_title):
        """Change the time unit for the x-axis of the given graph."""
        current_label = ax.get_xlabel()
        if "ms" in current_label:
            ax.set_xlabel("Time (s)", fontsize=12, fontweight="bold", color="black")
            ax.set_title(graph_title + " (Seconds)", fontsize=14, fontweight="bold", color="black")
        elif "Minutes" in current_label:
            ax.set_xlabel("Time (ms)", fontsize=12, fontweight="bold", color="black")
            ax.set_title(graph_title + " (Milliseconds)", fontsize=14, fontweight="bold", color="black")
        else:
            ax.set_xlabel("Time (Minutes)", fontsize=12, fontweight="bold", color="black")
            ax.set_title(graph_title + " (Minutes)", fontsize=14, fontweight="bold", color="black")
        ax.figure.canvas.draw()

    def reset_all(self):
        """Reset everything to initial state"""
        # Stop recording if active
        if self.recording:
            self.stop_recording()
        # Disconnect port if connected
        if self.serial_inst.is_open:
            self.disconnect_serial()
        if self.serial_inst2.is_open:
            self.disconnect_serial2()
        # Reset all data arrays
        self.x_data.clear()
        self.y_data.clear()
        self.x_data2.clear()
        self.y_data2.clear()
        # Reset start times
        self.start_time = None
        self.start_time2 = None
        # Reset graphs
        self.clear_graph()
        self.clear_graph2()
        # Clear displays
        self.clear_display()
        # Reset comboboxes
        self.port_combobox.set('')
        self.baudrate_combobox.set('')
        self.port2_combobox.set('')
        self.baudrate2_combobox.set('')
        self.time_unit_combobox.set('Seconds')
        # Reset file path
        self.file_path = "received_data.csv"
        # Reset status
        self.time_info_label.config(text="Online Time: N/A", font=("Arial", 12), foreground="black")
        self.com_status_label.config(text="COM Port Status: Disconnected", foreground="red")
        
        # Clear any additional online time labels
        for widget in self.status_frame.grid_slaves():
            if isinstance(widget, ttk.Label) and "Online Time:" in widget.cget("text"):
                if widget != self.time_info_label:
                    widget.destroy()
        
        messagebox.showinfo("Reset Complete", "All settings and data have been reset to initial state.")

    def on_closing(self):
        if self.serial_inst.is_open:
            self.serial_inst.close()
        if self.serial_inst2.is_open:
            self.serial_inst2.close()
        try:
            self.file.close()
        except AttributeError:
            pass
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()


