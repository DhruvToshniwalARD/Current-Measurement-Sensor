#!/usr/bin/env python
# coding: utf-8

# In[1]:


from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import sys
import serial
import time
import numpy as np
from serial.tools import list_ports

class ArduinoData:
    def __init__(self):
        self.ser = None
        self.serial_connection = None
        self.com_port = None
        self.baud_rate = None  
        
    def clear_buffer(self):
        self.ser.flush()

    def connect(self, com_port, baud_rate):
        if self.ser and self.ser.isOpen():
            self.ser.close()
        self.ser = serial.Serial(com_port, baud_rate, timeout=1)
        self.baud_rate = baud_rate  # Store the baudrate
        time.sleep(2)
        
    def read_data(self):
        try:
            if self.ser:
                line = self.ser.readline().decode('utf-8').strip()  # Read a line of data
                if line:  # Check that the line is not empty
                    current_str, voltage_str = line.split(",")  # Split the line at the comma
                    current = float(current_str)  # Convert the current string to a float
                    voltage = float(voltage_str)  # Convert the voltage string to a float
                    return current, voltage
            return None, None
        except Exception as e:
            return None, None

    def send_data(self, data):
        if self.ser is not None and self.ser.isOpen():
            self.ser.write((data + '\n').encode())
        else:
            #self.raw_data_view.append('Error: Serial connection is not open')
            print('Error: Serial connection is not open')

    def close(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            
    def update_com_port(self, new_com_port):
        if self.ser and self.ser.isOpen():  # Check if the serial object exists and if the port is open
            self.ser.close()  # Close the current connection
        if new_com_port:  # Only create a serial connection if a valid COM port is given
            self.ser = serial.Serial(new_com_port, self.baud_rate, timeout=1)
        else:
            #self.raw_data_view.append('Error: Serial connection is not open')
            print("No COM port selected")  # Print a message if no valid COM port is given

    def refresh_com_ports(self):
        comlist = serial.tools.list_ports.comports()
        available_ports = [comport.device for comport in comlist]
        return available_ports

    def update_baud_rate(self, new_baud_rate):
        if self.ser is not None and self.ser.isOpen():
            current_com_port = self.ser.port
            self.ser.close()  # Close the current connection
            self.ser = serial.Serial(current_com_port, new_baud_rate, timeout=1)
            self.baud_rate = new_baud_rate  # Update the stored baudrate

    @staticmethod
    def get_available_com_ports():
        return [port.device for port in list_ports.comports()]
    
    def disconnect(self):
        if self.ser.is_open:
            self.ser.close()
            
    def isConnected(self):
        return self.ser is not None and self.ser.isOpen()


# In[2]:


from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QScrollBar
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from pyqtgraph import PlotWidget as pgPlotWidget, SignalProxy, InfiniteLine, TextItem, ViewBox
import numpy as np
import pyqtgraph as pg

class CustomViewBox(ViewBox):
    def __init__(self, graph_window, *args, **kwargs):
        self.graph_window = graph_window
        super().__init__(*args, **kwargs)

    def wheelEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ShiftModifier:
            # Shift key pressed. Zoom y-axis.
            self.setMouseEnabled(x=False, y=True)
            if event.delta() > 0:  # If the wheel event is scrolling up
                if self.graph_window.y_label.lower() == 'current':
                    self.graph_window.app_window.zoom_in_yc()  # Call the zoom_in method for current
                else:
                    self.graph_window.app_window.zoom_in_yv()  # Call the zoom_in method for voltage
            else:  # If the wheel event is scrolling down
                if self.graph_window.y_label.lower() == 'current':
                    self.graph_window.app_window.zoom_out_yc()  # Call the zoom_out method for current
                else:
                    self.graph_window.app_window.zoom_out_yv()  # Call the zoom_out method for voltage
        else:
            # Shift key not pressed. Zoom x-axis.
            self.setMouseEnabled(x=True, y=False)
        super().wheelEvent(event)  # Pass the event to the PlotWidget

        if self.graph_window:
            self.graph_window.update_zoom_level()

class CustomPlotWidget(pgPlotWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def mousePressEvent(self, event):
        self.getPlotItem().getViewBox().setMouseEnabled(y=True)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.getPlotItem().getViewBox().setMouseEnabled(y=False)
        super().mouseReleaseEvent(event)

class SyncedScrollBar(QScrollBar):
    valueChangedSync = pyqtSignal(int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def slide(self, value):
        self.setValue(value)
        self.valueChangedSync.emit(value)

    def setValue(self, value):
        super().setValue(value)
        self.valueChangedSync.emit(value)

class GraphWindow(QMainWindow):
    def __init__(self, app_window, title, x, y, x_label, y_label, other=None):
        super().__init__()

        self.setWindowTitle(title)
        self.app_window = app_window

        self.layout = QVBoxLayout()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(self.layout)

        self.view_box = CustomViewBox(self)
        self.graphWidget = CustomPlotWidget(viewBox=self.view_box)
        self.layout.addWidget(self.graphWidget)

        # Synchronized Scroll Bar
        self.scrollbar = SyncedScrollBar(Qt.Horizontal)
        self.layout.addWidget(self.scrollbar)

        color = 'y' if y_label.lower() == 'current' else 'r'
        self.data_line = self.graphWidget.plot(x, y, name=y_label, pen=pg.mkPen(color))
        #self.set_y_limits()

        units = 'mA' if y_label.lower() == 'current' else 'mV'
        self.graphWidget.setLabel('left', y_label, units=units)
        self.graphWidget.setLabel('bottom', x_label, units='s')
        self.y_label = y_label

        self.vLine = pg.InfiniteLine(angle=90, movable=False)  # This creates the line
        self.graphWidget.addItem(self.vLine, ignoreBounds=True)  # This adds the line to the plot

        # Create a TextItem for displaying the y value on the graph
        self.y_value_text = pg.TextItem(anchor=(0, 1))
        self.graphWidget.addItem(self.y_value_text)

        self.other = other
        self.proxy = SignalProxy(self.graphWidget.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)

        self.x = x
        self.y = y
        self.graphWidget.getPlotItem().getViewBox().setMouseEnabled(y=False)  # Disable y-axis zoom
        # Show the grid with maximum opacity
        self.graphWidget.showGrid(x=True, y=True, alpha=1)
        
        self.scrollbar.setRange(int(min(self.x)) if self.x else 0, int(max(self.x)) if self.x else 0)
        self.scrollbar.valueChangedSync.connect(self.update_scroll)
        self.scrollbar.valueChanged.connect(self.update_plot_range)

        if self.other is not None:
            self.other.scrollbar.valueChangedSync.connect(self.sync_scroll)
            
    def set_y_limits(self, margin=0.1):
        y_min = min(self.y)
        y_max = max(self.y)
        y_range = y_max - y_min
        self.graphWidget.setYRange(y_min, y_max + margin * y_range)  # add a margin to the y-range

    def set_y_limits(self):
        self.graphWidget.getPlotItem().getViewBox().setLimits(yMin=min(self.y), yMax=max(self.y))

    def get_x_range(self):
        return self.graphWidget.viewRange()[0]
    
    def wheelEvent(self, event):
        self.graphWidget.wheelEvent(event)  # Pass the event to the PlotWidget
        self.update_zoom_level()  # Up

    def update_zoom_level(self):
        min_x, max_x = self.get_x_range()
        range_x = max_x - min_x
        self.app_window.zoom_level_x = range_x  # update zoom level
 
    def mouseMoved(self, evt):
        try:
            pos = evt[0]  # Using signal proxy turns original arguments into a tuple
            if self.graphWidget.sceneBoundingRect().contains(pos):
                mousePoint = self.graphWidget.plotItem.vb.mapSceneToView(pos)
                self.vLine.setPos(mousePoint.x())
                if not np.isnan(mousePoint.x()):
                    mouse_x = mousePoint.x()
                    index = (np.abs(self.x - mouse_x)).argmin()
                    if 0 <= index < len(self.x):
                        # Add a semi-transparent background to the label
                        self.y_value_text.setHtml(f'<span style="color: white; background-color: rgba(0, 0, 0, 0.5)">{self.y_label}:{self.y[index]:.2f}</span>')  # Update this graph's y value
                        
                        # Add some padding to the y position to keep the label within the window
                        y_position = self.y[index] - (max(self.y) - min(self.y)) * 0.05
                        self.y_value_text.setPos(mousePoint.x(), y_position)
                        
                        if self.other:
                            # Add a semi-transparent background to the label
                            self.other.y_value_text.setHtml(f'<span style="color: white; background-color: rgba(0, 0, 0, 0.5)">{self.other.y_label}:{self.other.y[index]:.2f}</span>')  # Update other graph's y value

                            # Add some padding to the y position to keep the label within the window
                            y_position_other = self.other.y[index] - (max(self.other.y) - min(self.other.y)) * 0.05
                            self.other.y_value_text.setPos(mousePoint.x(), y_position_other)
                            
                            self.other.vLine.setPos(mousePoint.x())  # This moves the line in the other graph
        except IndexError as e:
            print(f"IndexError occurred: {e}")
            print(f"mousePoint.x() = {mousePoint.x()}")
            print(f"len(self.x) = {len(self.x)}")
            print(f"len(self.y) = {len(self.y)}")
            if self.other:
                print(f"len(self.other.y) = {len(self.other.y)}")

    def update_plot_data(self, x, y):
        self.data_line.setData(x, y)
        self.x = x
        self.y = y
        #if(self.x):
        self.scrollbar.setRange(int(min(self.x)), int(max(self.x)))  # Update the scroll bar's range

    def sync_scroll(self, value):
        self.scrollbar.slide(value)

    @pyqtSlot(int)
    def update_scroll(self, value):
        self.graphWidget.setXRange(value, value + self.app_window.zoom_level_x)

    def update_plot_range(self, value):
        self.graphWidget.setXRange(value, value + self.app_window.zoom_level_x)
        if self.other:
            self.other.graphWidget.setXRange(value, value + self.other.app_window.zoom_level_x)


# In[3]:


from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QPushButton, QSlider, QMessageBox, QTextEdit, QFileDialog, QComboBox, QLineEdit, QCheckBox, QHBoxLayout, QGroupBox, QDesktopWidget, QDialog
from PyQt5.QtCore import QTimer, Qt, QSettings
from pyqtgraph import PlotWidget, exporters
import pyqtgraph as pg
import sys
import numpy as np
import csv
import pandas as pd
from datetime import datetime
import json
import os
import shutil

class AppWindow(QMainWindow):
    def __init__(self, arduino, *args, **kwargs):
        
        super(AppWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("Current Sensor Tool")  # The window title is set
        self.arduino = arduino  # The arduino object is passed to the class

        # The layout is set up
        self.layout = QVBoxLayout()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(self.layout)
        self.zoom_level_x = 100  # A zoom level variable is initialized
        self.zoom_level_y_current = 10
        self.zoom_level_y_voltage = 5000
        self.sample_rate = None
        self.live_data_paused = False
        self.sample_rate_button_clicks = 1    
        self.log_file = None
        self.log_writer = None
        self.logging = False  # add this line
        self.backup_file_name = None
      
        self.raw_data_view = QTextEdit()
        # Add a QTextEdit widget for the data field
        self.data_field = QTextEdit()
        #self.layout.addWidget(self.data_field)
        
        # Different buttons, fields, sliders and checkboxes are created and added to the layout
        self.save_battery_data_button = QPushButton('Save Battery Data')
        self.save_battery_data_button.setStyleSheet("background-color: lightgreen")
        self.save_battery_data_button.clicked.connect(self.save_battery_data)

        #self.layout.addWidget(self.save_battery_data_button)
                
        # Sample rate field and button
        self.sample_rate_group = QGroupBox("Sample Rate:")

        self.sample_rate_field = QLineEdit()
        self.sample_label = QLabel('Sample Rate:')
        #self.layout.addWidget(self.sample_label)
        #self.layout.addWidget(self.sample_rate_field)
        self.sample_rate_units_combo = QComboBox()
        self.sample_rate_units_combo.addItems(['ms', 's'])
        #self.layout.addWidget(self.sample_rate_units_combo)
        self.sample_rate_button = QPushButton('Set Sample Rate')
        self.sample_rate_button.setStyleSheet("background-color: lightyellow")
        self.sample_rate_button.clicked.connect(self.set_sample_rate)
        #self.layout.addWidget(self.sample_rate_button)
        
        self.sample_rate_options = QHBoxLayout()
        self.sample_rate_options.addWidget(self.sample_rate_field)
        self.sample_rate_options.addWidget(self.sample_rate_units_combo)
        self.sample_rate_options.addWidget(self.sample_rate_button)
        
        self.sample_rate_group.setLayout(self.sample_rate_options)
        #self.layout.addWidget(self.sample_rate_group)
        
        self.data_group = QGroupBox("Graphing Control")
        # Acquire data layout
        self.acquire_start_button = QPushButton('Start Acquisition')
        self.acquire_start_button.setStyleSheet("background-color: lightpink")
        self.acquire_start_button.clicked.connect(self.start_data_collection)

        self.acquire_stop_button = QPushButton('Stop Acquisition')
        self.acquire_stop_button.setStyleSheet("background-color: lightpink")
        self.acquire_stop_button.clicked.connect(self.stop_data_collection)

        # Live data layout
        self.pause_button = QPushButton('Pause Display')
        self.pause_button.setStyleSheet("background-color: lightpink")
        self.pause_button.clicked.connect(self.toggle_live_data)
        
        #Clear and sync data layout
        self.clear_button = QPushButton('Clear Data')
        self.clear_button.setStyleSheet("background-color: lightcoral")
        self.clear_button.clicked.connect(self.clear_data)
        
        self.sync_button = QPushButton('Sync Graphs')
        self.sync_button.setStyleSheet("background-color: lightpink")
        self.sync_button.clicked.connect(self.sync_graphs)

        self.data_control = QVBoxLayout()
        self.data_acquire = QHBoxLayout()
        self.live_data = QHBoxLayout()
        self.clear_and_sync = QHBoxLayout()

        self.data_acquire.addWidget(self.acquire_start_button)
        self.data_acquire.addWidget(self.acquire_stop_button)
        self.live_data.addWidget(self.pause_button)
        self.clear_and_sync.addWidget(self.clear_button)
        self.clear_and_sync.addWidget(self.sync_button)

        self.data_control.addLayout(self.data_acquire)  # add first horizontal layout to the parent layout
        self.data_control.addLayout(self.live_data)  # add second horizontal layout to the parent layout
        self.data_control.addLayout(self.clear_and_sync)  # add second horizontal layout to the parent layout

        self.data_group.setLayout(self.data_control)  # Set the group layout to the parent layout
        self.layout.addWidget(self.data_group)  # Add the group

        self.save_datas = QPushButton('Save Data')
        self.save_datas.setStyleSheet("background-color: lightgreen")
        self.save_datas.clicked.connect(self.save_data)
        self.save_image_button = QPushButton('Save Image')
        self.save_image_button.setStyleSheet("background-color: lightgreen")
        self.save_image_button.clicked.connect(self.save_image)

        self.save_raw_data_button = QPushButton('Save Raw Data')
        self.save_raw_data_button.setStyleSheet("background-color: lightgreen")
        self.save_raw_data_button.clicked.connect(self.save_raw_data)

        self.load_button = QPushButton('Load Data')
        self.load_button.setStyleSheet("background-color: lightblue")
        self.load_button.clicked.connect(self.load_data)
        
        self.saveandloadlayout = QHBoxLayout()
        self.saveandloadlayout.addWidget(self.save_datas)
        self.saveandloadlayout.addWidget(self.load_button)
        
        self.saveimageandrawdatalayout = QHBoxLayout()
        self.saveimageandrawdatalayout.addWidget(self.save_image_button)
        self.saveimageandrawdatalayout.addWidget(self.save_raw_data_button)

        # Add the horizontal layout to the main layout
        self.layout.addLayout(self.saveandloadlayout)
        self.layout.addLayout(self.saveimageandrawdatalayout)
        
        self.zoom_in_x_button = QPushButton('t +')
        self.zoom_in_x_button.clicked.connect(self.zoom_in_x)

        self.zoom_out_x_button = QPushButton('t -')
        self.zoom_out_x_button.clicked.connect(self.zoom_out_x)
        
        self.zoom_max_x_button = QPushButton('t Max out')
        self.zoom_max_x_button.clicked.connect(self.zoom_max_x)  # Connect to the new zoom_max_x method
        
        self.zoom_in_yc_button = QPushButton('C +')
        self.zoom_in_yc_button.clicked.connect(self.zoom_in_yc)

        self.zoom_out_yc_button = QPushButton('C - out')
        self.zoom_out_yc_button.clicked.connect(self.zoom_out_yc)

        self.zoom_max_yc_button = QPushButton('C Max out')
        self.zoom_max_yc_button.clicked.connect(self.zoom_max_yc)
        
        self.zoom_in_yv_button = QPushButton('V +')
        self.zoom_in_yv_button.clicked.connect(self.zoom_in_yv)

        self.zoom_out_yv_button = QPushButton('V - out')
        self.zoom_out_yv_button.clicked.connect(self.zoom_out_yv)

        self.zoom_max_yv_button = QPushButton('V Max out')
        self.zoom_max_yv_button.clicked.connect(self.zoom_max_yv)
        
        # Add a border that encompasses Connect, Refresh, COM Port and Baud Rate
        self.zoom_group = QGroupBox("Zoom Settings")
        self.zoom_layout_h1 = QHBoxLayout()
        self.zoom_layout_h2 = QHBoxLayout()
        self.zoom_layout_h3 = QHBoxLayout()
        self.zoom_layout_v = QVBoxLayout()  # parent layout

        self.zoom_layout_h1.addWidget(self.zoom_in_x_button)
        self.zoom_layout_h1.addWidget(self.zoom_out_x_button)
        self.zoom_layout_h1.addWidget(self.zoom_max_x_button)
        self.zoom_layout_h2.addWidget(self.zoom_in_yc_button)
        self.zoom_layout_h2.addWidget(self.zoom_out_yc_button)
        self.zoom_layout_h2.addWidget(self.zoom_max_yc_button)
        self.zoom_layout_h3.addWidget(self.zoom_in_yv_button)
        self.zoom_layout_h3.addWidget(self.zoom_out_yv_button)
        self.zoom_layout_h3.addWidget(self.zoom_max_yv_button)

        self.zoom_layout_v.addLayout(self.zoom_layout_h1)  # add first horizontal layout to the parent layout
        self.zoom_layout_v.addLayout(self.zoom_layout_h2)  # add second horizontal layout to the parent layout
        self.zoom_layout_v.addLayout(self.zoom_layout_h3)  # add second horizontal layout to the parent layout

        self.zoom_group.setLayout(self.zoom_layout_v)  # Set the group layout to the parent layout
        self.layout.addWidget(self.zoom_group)  # Add the group

        
        # Add the exit button
        self.exit_button = QPushButton('Exit')
        self.exit_button.setStyleSheet("background-color: lightcoral")
        self.exit_button.clicked.connect(self.close_application)        
        self.layout.addWidget(self.raw_data_view)
        
        # Similarly, create a horizontal layout for the connect and refresh connection buttons
        self.connect_refresh_layout = QHBoxLayout()
        self.connect_button = QPushButton('Connect')
        self.connect_button.setStyleSheet("background-color: lightyellow")
        self.connect_button.clicked.connect(lambda: self.disconnect_to_arduino() if self.connect_button.text() == "Disconnect" else self.connect_to_arduino())
        self.connect_refresh_layout.addWidget(self.connect_button)
        self.refresh_button = QPushButton('Refresh Connection')
        self.refresh_button.setStyleSheet("background-color: lightyellow")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_refresh_layout.addWidget(self.refresh_button)

        # Add a border that encompasses Connect, Refresh, COM Port and Baud Rate
        self.connection_group = QGroupBox("Connection Settings")
        self.connection_layout = QVBoxLayout()  # Create a new layout for the group

        # Add the 'Sample Rate' layout to the 'Connection Settings' layout
        self.connection_layout.addWidget(self.sample_rate_group)


        # Layout for COM port
        self.com_layout = QHBoxLayout()
        self.com_label = QLabel('COM Port:')
        self.com_combo = QComboBox()
        self.com_combo.addItems(self.arduino.get_available_com_ports())
        self.com_combo.currentIndexChanged.connect(self.update_com_port)
        self.com_layout.addWidget(self.com_label)
        self.com_layout.addWidget(self.com_combo)

        # Layout for Baud rate
        self.baud_layout = QHBoxLayout()
        self.baud_label = QLabel('Baud Rate:')
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '14400', '19200', '38400', '57600', '115200'])
        self.baud_combo.currentIndexChanged.connect(self.update_baud_rate)
        self.baud_layout.addWidget(self.baud_label)
        self.baud_layout.addWidget(self.baud_combo)

        self.connection_layout.addLayout(self.com_layout)  # Add the com layout to the group layout
        self.connection_layout.addLayout(self.baud_layout)  # Add the baud layout to the group layout
        self.connection_layout.addLayout(self.connect_refresh_layout)  # Add the connect/refresh layout to the group layout
        self.connection_group.setLayout(self.connection_layout)  # Set the group layout
        self.layout.addWidget(self.connection_group)  # Add the group
        
        # Create a horizontal layout
        self.saveconfigandloadconfiglayout = QHBoxLayout()

        self.save_config_button = QPushButton('Save Config')
        self.save_config_button.setStyleSheet("background-color: lightgreen")
        self.save_config_button.clicked.connect(self.save_config)
        self.saveconfigandloadconfiglayout.addWidget(self.save_config_button)

        self.load_config_button = QPushButton('Load Config')
        self.load_config_button.setStyleSheet("background-color: lightblue")
        self.load_config_button.clicked.connect(self.load_config)
        self.saveconfigandloadconfiglayout.addWidget(self.load_config_button)

        # Add the horizontal layout to the main layout
        self.layout.addLayout(self.saveconfigandloadconfiglayout)

        self.layout.addWidget(self.exit_button)
        
        self.save_battery_data_button.setToolTip('Click to save the battery data')
        self.sample_rate_button.setToolTip('Click to set the sample rate')
        self.acquire_start_button.setToolTip('Click to start data acquisition')
        self.acquire_stop_button.setToolTip('Click to stop data acquisition')
        self.pause_button.setToolTip('Click to pause/unpause graph')
        self.clear_button.setToolTip('Click to clear data')
        self.sync_button.setToolTip('Click to sync the graphs')
        self.save_image_button.setToolTip('Click to save an image of the graphs')
        self.save_raw_data_button.setToolTip('Click to save raw data')
        self.load_button.setToolTip('Click to load data')
        self.zoom_in_x_button.setToolTip('Click to zoom in on the X axis')
        self.zoom_out_x_button.setToolTip('Click to zoom out on the X axis')
        self.zoom_max_x_button.setToolTip('Click to maximize the X axis')
        self.zoom_in_yc_button.setToolTip('Click to zoom in on the Current Y axis')
        self.zoom_out_yc_button.setToolTip('Click to zoom out on the Current Y axis')
        self.zoom_max_yc_button.setToolTip('Click to maximize the Current Y axis')
        self.zoom_in_yv_button.setToolTip('Click to zoom in on the Voltage Y axis')
        self.zoom_out_yv_button.setToolTip('Click to zoom out on the Voltage Y axis')
        self.zoom_max_yv_button.setToolTip('Click to maximize the Voltage Y axis')
        self.exit_button.setToolTip('Click to exit the application')
        self.connect_button.setToolTip('Click to connect/disconnect to the Arduino')
        self.refresh_button.setToolTip('Click to refresh the COM ports')
        self.save_config_button.setToolTip('Click to save the current configuration')
        self.load_config_button.setToolTip('Click to load a configuration')


        self.x = []
        self.y_current = []
        self.y_voltage = []

        self.settings = QSettings('YourCompany', 'YourApp')
        last_config_file = self.settings.value('last_config_file', '')
        if last_config_file:
            self.load_config(last_config_file)
    
    
        # Two GraphWindows are created for displaying the current and voltage respectively
        self.graphWindow1 = GraphWindow(self, "Current", self.x, self.y_current, 'Time', 'Current', None)
        self.graphWindow2 = GraphWindow(self, "Voltage", self.x, self.y_voltage, 'Time', 'Voltage', self.graphWindow1)

        self.graphWindow1.other = self.graphWindow2
        self.graphWindow2.other = self.graphWindow1
        
        screen_resolution = QApplication.desktop().screenGeometry()
        width, height = screen_resolution.width(), screen_resolution.height()
        self.setGeometry(int(width*0.80), 0, int(width*0.20), int(height*0.95))
        self.graphWindow1.resize(int(width*0.80), int(height*0.45))
        self.graphWindow2.resize(int(width*0.80), int(height*0.45))

        self.graphWindow1.move(0, 0)
        self.graphWindow2.move(0, int(height*0.45))
        # A QTimer object is created
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)   
        
# Various functions are defined for different operations like saving/loading data, starting/stopping data collection, and more

    def save_config(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Config", "", "JSON Files (*.json);;All Files (*)", options=options)
        if fileName:
            config = {
                "sample_rate": self.sample_rate_field.text(),
                "sample_rate_units": self.sample_rate_units_combo.currentText(),
                "com_port": self.com_combo.currentText(),
                "baud_rate": self.baud_combo.currentText()
            }
            with open(fileName, 'w') as json_file:
                json.dump(config, json_file)
            self.settings.setValue('last_config_file', fileName)
            
    def toggle_autoscale(self, state):
        if state == Qt.Checked:
            self.graphWindow1.graphWidget.getViewBox().setAutoVisible(y=True)
            self.graphWindow2.graphWidget.getViewBox().setAutoVisible(y=True)
        else:
            self.graphWindow1.graphWidget.getViewBox().setAutoVisible(y=False)
            self.graphWindow2.graphWidget.getViewBox().setAutoVisible(y=False)

    def load_config(self, fileName=None):
        if not fileName:
            options = QFileDialog.Options()
            options |= QFileDialog.ReadOnly
            fileName, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "JSON Files (*.json);;All Files (*)", options=options)
        if fileName and os.path.exists(fileName):
            with open(fileName, 'r') as json_file:
                config = json.load(json_file)
                self.sample_rate_field.setText(config["sample_rate"])
                self.sample_rate_units_combo.setCurrentText(config["sample_rate_units"])
                self.com_combo.setCurrentText(config["com_port"])
                self.baud_combo.setCurrentText(config["baud_rate"])
                self.connect_to_arduino()
                self.set_sample_rate()

    def refresh_ports(self):
        new_ports = self.arduino.refresh_com_ports()
        self.com_combo.clear()
        self.com_combo.addItems(new_ports)

    def connect_to_arduino(self):
        selected_com_port = self.com_combo.currentText()
        selected_baud_rate = int(self.baud_combo.currentText())
        try:
            self.arduino.connect(selected_com_port, selected_baud_rate)
            self.raw_data_view.append(f'Connected to {selected_com_port} at {selected_baud_rate} baud rate.')
            self.connect_button.setText("Disconnect")  # Change the button text
        except serial.SerialException as e:
            self.raw_data_view.append(f'Error connecting to the Current Sensor: {str(e)}')
        except Exception as e:
            self.raw_data_view.append(str(e))         
            
    def toggle_live_data(self):
        if self.live_data_paused:
            self.pause_button.setText('Pause Display')
            self.live_data_paused = False
        else:
            self.pause_button.setText('Unpause Display')
            self.live_data_paused = True

    def disconnect_to_arduino(self):
        try:
            self.arduino.disconnect() 
            self.connect_button.setText("Connect")  # Change the button text back to "Connect"
            self.timer.stop()  # Stop the timer when disconnecting
        except Exception as e:
            self.raw_data_view.append(str(e))
            
    def save_data(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Data", "", "CSV Files (*.csv);;All Files (*)", options=options)
        if fileName:
            if not fileName.endswith('.csv'):
                fileName += '.csv'
            if self.backup_file_name:
                shutil.copyfile(self.backup_file_name, fileName)
  
    def start_data_collection(self):
        if self.arduino.isConnected():  # Checks if Arduino is connected
            if not self.timer.isActive():  # Only start the timer if it's not already running
                self.arduino.send_data("Start")
                self.timer.start(0)  # update every 0 ms
                self.graphWindow1.show()
                self.graphWindow2.show()
                self.start_logging()
                self.sample_rate_button.setEnabled(False)  # Disable the "Set Sample Rate" button
        else:
            self.raw_data_view.append("Current Sensor is not connected.")
            
    def start_logging(self):
        if not self.backup_file_name:
            timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
            self.backup_file_name = f'CMS_{timestamp}_backup.csv'
            with open(self.backup_file_name, 'w', newline='') as f:
                log_writer = csv.writer(f)
                log_writer.writerow(['Sample Rate: ' + str(self.sample_rate), 'Date and Time: ' + datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
                log_writer.writerow(['Current', 'Voltage'])

    def start_live_data_collection(self):
        self.arduino.clear_buffer()
        if self.arduino.isConnected():  # Checks if Arduino is connected
            if not self.timer.isActive():  # Only start the timer if it's not already running
                self.arduino.send_data("Start")
                self.timer.start(0)  # update every 0 ms
                self.graphWindow1.show()
                self.graphWindow2.show()
                self.sample_rate_button.setEnabled(False)  # Disable the "Set Sample Rate" button
        else:
            self.raw_data_view.append("Current Sensor is not connected.")
            
    def save_backup(self):
        if self.backup_file_name:
            # Use a predefined directory
            backup_directory = os.path.expanduser("~")
            # Create the backup file path with a '_backup' suffix
            backup_file_name = os.path.splitext(self.backup_file_name)[0] + '_backup.csv'
            backup_file_path = os.path.join(backup_directory, backup_file_name)
            # Copy the backup file to the directory
            shutil.copyfile(self.backup_file_name, backup_file_path)

    def stop_data_collection(self):
        self.arduino.send_data("Stop")
        self.timer.stop()
        self.arduino.clear_buffer()
        self.sample_rate_button.setEnabled(True)  # Disable the "Set Sample Rate" button

    def update(self):
        try:
            data = self.arduino.read_data()  # Using the instance variable
            current, voltage = data
            if current is not None and voltage is not None:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # get the current time with milliseconds
                self.update_plot_data('current', current)
                self.update_plot_data('voltage', voltage)
                self.raw_data_view.append(f'{timestamp} - I: {current} - V: {voltage}')  # Update the raw data view
                if self.backup_file_name:   # If logging has started, write to the log
                    with open(self.backup_file_name, 'a', newline='') as f:
                        log_writer = csv.writer(f)
                        log_writer.writerow([current, voltage])
        except Exception as e:
            print(f"Error in update: {e}")

    def sync_graphs(self):
        # Get the x-ranges of the two graphs
        x_range1 = self.graphWindow1.get_x_range()
        x_range2 = self.graphWindow2.get_x_range()

        # Find the intersection of the two x-ranges
        min_x = max(x_range1[0], x_range2[0])
        max_x = min(x_range1[1], x_range2[1])

        # Set the x-axes of both graphs to the intersection
        self.graphWindow1.graphWidget.setXRange(min_x, max_x)
        self.graphWindow2.graphWidget.setXRange(min_x, max_x)

    def load_data(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        fileName, _ = QFileDialog.getOpenFileName(self, "Load Data", "", "CSV Files (*.csv);;All Files (*)", options=options)
        if fileName:
            with open(fileName, 'r') as f:
                reader = csv.reader(f)
                header1 = next(reader)  # read the first row
                sample_rate_str = header1[0]  # sample rate is in the first column
                sample_rate_val = sample_rate_str.split(':')[1].strip()  # split the string and take the second part
                self.sample_rate = int(sample_rate_val)  # update the sample_rate value
                next(reader)  # skip the second row (headers)

                # read the remaining rows into lists
                self.y_current = []
                self.y_voltage = []
                for row in reader:
                    self.y_current.append(float(row[0]))
                    self.y_voltage.append(float(row[1]))

            # convert lists to numpy arrays
            self.y_current = np.array(self.y_current)
            self.y_voltage = np.array(self.y_voltage)

            # calculate the time values based on the sample rate
            self.x = np.arange(0, len(self.y_current)) * (self.sample_rate / 2000)  # convert sample rate from ms to s and divide by 2

            # check if graph windows are open, if not, open them
            if self.graphWindow1 is None or not self.graphWindow1.isVisible():
                self.graphWindow1.show()
            if self.graphWindow2 is None or not self.graphWindow2.isVisible():
                self.graphWindow2.show()

            self.graphWindow1.update_plot_data(self.x, self.y_current)
            self.graphWindow2.update_plot_data(self.x, self.y_voltage)
            
    def set_sample_rate(self):
        self.sample_rate_button_clicks += 1
        if self.sample_rate_button_clicks % 2 == 1:  # Display the warning on the 1st, 3rd, 5th, etc. clicks
            QMessageBox.warning(self, "Warning", "If you have previous data, save it and then set the sample rate again.")
            return

        sample_rate_value = self.sample_rate_field.text().strip()  # Remove leading/trailing whitespace
        sample_rate_units = self.sample_rate_units_combo.currentText()
        if self.arduino.isConnected():  # Checks if Arduino is connected
            if not sample_rate_value:  # If the sample rate field is empty
                self.raw_data_view.append('Error: Please enter a sample rate.')
                return

            try:
                sample_rate_value = int(sample_rate_value)  # Attempt to convert to integer
                if sample_rate_value <= 0:  # Check if the value is a positive integer
                    raise ValueError
            except ValueError:  # If the value is not a valid integer
                self.raw_data_view.append('Error: Invalid sample rate value. Please enter a positive number.')
                return

            if sample_rate_units == 's':
                sample_rate_value *= 1000  # convert to ms
            self.sample_rate = sample_rate_value
            self.raw_data_view.append('Sample rate set')

            self.arduino.send_data('s' + str(sample_rate_value))
        else:
            self.raw_data_view.append('Current Sensor is not connected')

    def update_plot_data(self, data_type, data_val):
        try:
            if not self.sample_rate:  # If the sample rate is not set, return
                return
            sample_rate_s = self.sample_rate / 2000  # Convert sample rate from ms to s and divide by 2
            time_value = (self.x[-1] + sample_rate_s) if len(self.x) > 0 else 0
            self.x = np.append(self.x, time_value)

            if data_type == 'current':
                self.y_current = np.append(self.y_current, data_val)
                self.y_voltage = np.append(self.y_voltage, np.nan)  # append np.nan to keep the same length as self.x
            elif data_type == 'voltage':
                self.y_voltage = np.append(self.y_voltage, data_val)
                self.y_current = np.append(self.y_current, np.nan)  # append np.nan to keep the same length as self.x

            # Interpolate missing data
            valid_current_indices = [i for i, y in enumerate(self.y_current) if np.isfinite(y)]
            valid_voltage_indices = [i for i, y in enumerate(self.y_voltage) if np.isfinite(y)]
            if valid_current_indices:
                self.y_current = np.interp(self.x, [self.x[i] for i in valid_current_indices], [self.y_current[i] for i in valid_current_indices])
            if valid_voltage_indices:
                self.y_voltage = np.interp(self.x, [self.x[i] for i in valid_voltage_indices], [self.y_voltage[i] for i in valid_voltage_indices])

            if not self.live_data_paused:  # If live data is not paused, update the plot
                self.graphWindow1.update_plot_data(self.x, self.y_current)
                self.graphWindow2.update_plot_data(self.x, self.y_voltage)

                # Dynamic adjustment of x-axis range based on latest data points
                min_x = self.x[-1] - self.zoom_level_x
                max_x = self.x[-1]
                self.graphWindow1.graphWidget.setXRange(min_x, max_x)
                self.graphWindow2.graphWidget.setXRange(min_x, max_x)

        except IndexError:
            pass

    def zoom_in_x(self):
        min_x, max_x = self.graphWindow1.graphWidget.viewRange()[0]
        range_x = max_x - min_x
        self.zoom_level_x = range_x / 2  # update zoom level
        self.graphWindow1.graphWidget.setXRange(min_x + range_x / 4, max_x - range_x / 4)
        self.graphWindow2.graphWidget.setXRange(min_x + range_x / 4, max_x - range_x / 4)

    def zoom_out_x(self):
        min_x, max_x = self.graphWindow1.graphWidget.viewRange()[0]
        range_x = max_x - min_x
        self.zoom_level_x = range_x * 2  # update zoom level
        self.graphWindow1.graphWidget.setXRange(min_x - range_x / 2, max_x + range_x / 2)
        self.graphWindow2.graphWidget.setXRange(min_x - range_x / 2, max_x + range_x / 2)

    def zoom_max_x(self):
        min_x = min(self.x)
        max_x = max(self.x)
        self.zoom_level_x = max_x - min_x  # update zoom level
        self.graphWindow1.graphWidget.setXRange(min_x, max_x)
        self.graphWindow2.graphWidget.setXRange(min_x, max_x)

    def zoom_in_yc(self):
        min_y, max_y = self.graphWindow1.graphWidget.viewRange()[1]
        range_y = max_y - min_y
        self.zoom_level_y_current = range_y / 2  # update zoom level
        self.graphWindow1.graphWidget.setYRange(min_y + range_y / 4, max_y - range_y / 4)

    def zoom_out_yc(self):
        min_y, max_y = self.graphWindow1.graphWidget.viewRange()[1]
        range_y = max_y - min_y
        self.zoom_level_y_current = range_y * 2  # update zoom level
        self.graphWindow1.graphWidget.setYRange(min_y - range_y / 2, max_y + range_y / 2)

    def zoom_in_yv(self):
        min_y, max_y = self.graphWindow2.graphWidget.viewRange()[1]
        range_y = max_y - min_y
        self.zoom_level_y_voltage = range_y / 2  # update zoom level
        self.graphWindow2.graphWidget.setYRange(min_y + range_y / 4, max_y - range_y / 4)

    def zoom_out_yv(self):
        min_y, max_y = self.graphWindow2.graphWidget.viewRange()[1]
        range_y = max_y - min_y
        self.zoom_level_y_voltage = range_y * 2  # update zoom level
        self.graphWindow2.graphWidget.setYRange(min_y - range_y / 2, max_y + range_y / 2)

    def zoom_max_yc(self):
        min_y_current = min(self.y_current)
        max_y_current = max(self.y_current)
        self.zoom_level_y_current = max_y_current - min_y_current  # update zoom level
        self.graphWindow1.graphWidget.setYRange(min_y_current, max_y_current)

    def zoom_max_yv(self):
        min_y_voltage = min(self.y_voltage)
        max_y_voltage = max(self.y_voltage)
        self.zoom_level_y_voltage = max_y_voltage - min_y_voltage  # update zoom level
        self.graphWindow2.graphWidget.setYRange(min_y_voltage, max_y_voltage)

    def change_scale(self):
        scale = self.scale_slider.value()
        self.graphWidget1.setXRange(0, scale)
        self.graphWidget2.setXRange(0, scale)
        
    def save_battery_data(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Battery Data", "", "Text Files (*.txt);;All Files (*)", options=options)
        if fileName:
            with open(fileName, 'w') as f:
                f.write(self.data_field.toPlainText())

    def save_image(self):
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
        exporter = exporters.ImageExporter(self.graphWindow1.graphWidget.plotItem)
        exporter.export(f'current_plot_{timestamp}.png')
        exporter = exporters.ImageExporter(self.graphWindow2.graphWidget.plotItem)
        exporter.export(f'voltage_plot_{timestamp}.png')

    def open_battery_characterization_panel(self):
        self.battery_char_panel = BatteryCharacterizationPanel(self)
        self.battery_char_panel.show()
        
    def save_raw_data(self):
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S%f')[:-3]  # get the current time with milliseconds
        fileName = f'{timestamp}_raw_data_view.txt'  # create a filename using the timestamp
        try:
            with open(fileName, 'w') as file:
                file.write(self.raw_data_view.toPlainText())
            self.raw_data_view.append(f'Successfully saved raw data to {fileName}.')
        except Exception as e:
            self.raw_data_view.append(f'Error saving raw data: {str(e)}')

    def clear_data(self):
        self.x = []
        self.y_current = []  # re-initialize with zeros for current
        self.y_voltage = []  # re-initialize with zeros for voltage
        if self.x:  # only update the plot if x is not empty
            self.graphWindow2.update_plot_data(self.x, self.y_voltage)
            self.graphWindow1.update_plot_data(self.x, self.y_current)

    def update_com_port(self):
        selected_com_port = self.com_combo.currentText()
        print(f"Selected COM port: {selected_com_port}")
        self.arduino.update_com_port(selected_com_port)
     
    def update_baud_rate(self):
        selected_baud_rate = int(self.baud_combo.currentText())
        self.arduino.update_baud_rate(selected_baud_rate)
        
    def close_application(self):
        reply = QMessageBox.question(self, 'Exit Confirmation',
                                     'Are you sure you want to exit?', QMessageBox.Yes |
                                     QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            #self.save_backup()
            self.arduino.close()  # Close the serial port
            QApplication.quit()  # Close the application
        else:
            return
        
    def closeEvent(self, event):
        self.close_application()
        event.accept()  # This line is necessary to ensure the window actually closes


# In[4]:


from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QPushButton

class StartupWindow(QDialog):
    bcs_selected = pyqtSignal()

    def __init__(self, parent=None):
        super(StartupWindow, self).__init__(parent)
        
        # Set the window title
        self.setWindowTitle('Current Measurement and Battery Characterization Application')

        # Create buttons
        self.cms_button = QPushButton("CMS")
        self.bcs_button = QPushButton("BCS")
        
        # Add buttons to layout
        layout = QVBoxLayout()
        layout.addWidget(self.cms_button)
        layout.addWidget(self.bcs_button)
        self.setLayout(layout)

        # Connect buttons to handlers
        self.cms_button.clicked.connect(self.open_cms)
        self.bcs_button.clicked.connect(self.open_bcs)

    def open_cms(self):
        self.done(QDialog.Accepted)

    def open_bcs(self):
        self.bcs_selected.emit()
        self.close()


# In[5]:


from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QTabWidget, QWidget, QCheckBox, QGroupBox, QPushButton, QLabel, QLineEdit, QFileDialog

class BCSWindow(QMainWindow):
    def __init__(self, parent=None):
        super(BCSWindow, self).__init__(parent)

        self.setWindowTitle('Battery Characterization System')

        # Create the checkboxes and start/stop buttons
        self.create_battery_controls()

        # Create the tabs
        self.tabs = QTabWidget()
        for i in range(1, 5):
            self.tabs.addTab(self.create_tab('Battery {}'.format(i)), 'Battery {}'.format(i))

        # Add checkboxes and tabs to the window layout
        layout = QVBoxLayout()
        layout.addLayout(self.battery_controls_layout)
        layout.addWidget(self.tabs)

        # Set the window layout
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def create_battery_controls(self):
        self.battery_controls_layout = QVBoxLayout()  # Use QVBoxLayout instead of QHBoxLayout

        for i in range(1, 5):
            # Create checkbox and start/stop buttons for a battery
            check = QCheckBox('Battery {}'.format(i))
            start_button = QPushButton('Start')
            stop_button = QPushButton('Stop')

            # TODO: Connect start/stop buttons to handlers

            # Add checkbox and start/stop buttons to layout
            layout = QHBoxLayout()  # Use QHBoxLayout for each battery's controls
            layout.addWidget(check)
            layout.addWidget(start_button)
            layout.addWidget(stop_button)
            self.battery_controls_layout.addLayout(layout)

    def create_tab(self, name):
        # Create the group boxes
        config_box = QGroupBox('Battery Configuration Screen')
        graph_box = QGroupBox('Graphing Options')

        # Add controls to the configuration group box
        self.add_config_controls(config_box)

        # TODO: Add controls to the graphing options group box

        # Add the group boxes to the tab layout
        layout = QVBoxLayout()
        layout.addWidget(config_box)
        layout.addWidget(graph_box)

        # Create the tab and set its layout
        tab = QWidget()
        tab.setLayout(layout)
        return tab

    def add_config_controls(self, group_box):
        # Define the configuration fields
        fields = [
            'Battery Model Number',
            'Battery Serial Number',
            'Battery Capacity',
            'Battery Make',
            'Manufacturing Year',
            'Project Name',
            'Charging - Discharging Cycle',
            'Number of Batteries',
            'Battery Cutoff Voltage',
            'Full Battery Voltage',
            'End of Charge Current'
        ]

        # Create labels and text boxes for each field
        layout = QVBoxLayout()
        for field in fields:
            label = QLabel(field)
            text_box = QLineEdit()
            layout.addWidget(label)
            layout.addWidget(text_box)

        # Create the directory chooser
        dir_label = QLabel('Default Directory')
        dir_button = QPushButton('Choose Directory')
        dir_button.clicked.connect(self.choose_directory)
        layout.addWidget(dir_label)
        layout.addWidget(dir_button)

        group_box.setLayout(layout)

    def choose_directory(self):
        dir = QFileDialog.getExistingDirectory(self, 'Choose Directory')
        if dir:
            print('Chose directory: {}'.format(dir))  # TODO: Do something with the chosen directory


# In[ ]:


def main():
    arduino = ArduinoData()
    app = QApplication([])

    startup_window = StartupWindow()
    result = startup_window.exec()

    if result == QDialog.Accepted:
        window = AppWindow(arduino)
        window.show()
    else:
        window = BCSWindow()
        window.show()
    app.exec()

if __name__ == '__main__':
    main()


# In[ ]:




