#!/usr/bin/env python3
"""
Robot Mapping Live Viewer - InfluxDB Client
=============================================

Program do oglądania mapowania robota w czasie rzeczywistym z dowolnego komputera w sieci.
Łączy się z InfluxDB na Raspberry Pi i wyświetla live dane.

Wymagania:
- pip install influxdb-client influxdb matplotlib numpy tkinter Pillow requests

Użycie:
python robot_viewer.py

"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time
import json
import requests
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import logging

# Konfiguracja logowania
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# === KONFIGURACJA POŁĄCZENIA ===
DEFAULT_CONFIG = {
    "robot_ip": "192.168.1.14",  # IP Raspberry Pi z robotem
    "influxdb_url": "http://192.168.1.14:8086",
    "username": "pi",
    "password": "raspberry", 
    "org": "mapper",
    "bucket": "mapper",
    "robot_id": "robot_001",
    "refresh_interval": 2000  # ms
}

# === DATA CLASSES ===
@dataclass
class RobotState:
    x: float
    y: float
    angle: float
    mode: str
    timestamp: datetime

@dataclass
class TerrainPoint:
    x: int
    y: int
    distance: float
    confidence: float
    is_obstacle: bool
    timestamp: datetime

@dataclass
class ExplorationStats:
    coverage_percent: float
    distance_traveled: float
    obstacles_detected: int
    session_time_minutes: float
    exploration_efficiency: float
    frontiers_count: int

# === INFLUXDB CLIENT ===
class InfluxDBClient:
    def __init__(self, config):
        self.config = config
        self.client = None
        self.client_v1 = None
        self.query_api = None
        self.connected = False
        self.connection_type = None
        
    def connect(self):
        """Próbuj połączyć z InfluxDB (v1.x lub v2.x)"""
        try:
            # Próba InfluxDB 2.x
            from influxdb_client import InfluxDBClient as InfluxDBClientV2
            
            auth_token = f"{self.config['username']}:{self.config['password']}"
            self.client = InfluxDBClientV2(
                url=self.config['influxdb_url'],
                token='90jddHVMoo6nZSgc3hJkX8ZLrCcyjoyIw67xVg2viUVdhh9XXrS-QOZX6uTbaia3A75NPRFU5Y4WFPhKvgQrLA==',
                org=self.config['org']
            )
            
            self.query_api = self.client.query_api()
            self.client.ping()
            self.connected = True
            self.connection_type = "v2.x"
            logger.info("✅ Połączono z InfluxDB v2.x")
            return True
            
        except Exception as e:
            logger.warning(f"InfluxDB v2.x failed: {e}")
            
        try:
            # Próba InfluxDB 1.x
            from influxdb import InfluxDBClient as InfluxDBClientV1
            
            # Parsuj URL
            url = self.config['influxdb_url']
            if "://" in url:
                host = url.split("://")[1].split(":")[0]
                port = int(url.split(":")[-1]) if ":" in url.split("://")[1] else 8086
            else:
                host = url.split(":")[0]
                port = int(url.split(":")[1]) if ":" in url else 8086
            
            self.client_v1 = InfluxDBClientV1(
                host=host,
                port=port,
                username=self.config['username'],
                password=self.config['password'],
                database='robot_mapping'
            )
            
            # Test połączenia
            self.client_v1.ping()
            self.connected = True
            self.connection_type = "v1.x"
            logger.info("✅ Połączono z InfluxDB v1.x")
            return True
            
        except Exception as e:
            logger.error(f"InfluxDB v1.x failed: {e}")
            self.connected = False
            return False
    
    def get_latest_robot_state(self, robot_id: str) -> Optional[RobotState]:
        """Pobierz najnowszy stan robota"""
        if not self.connected:
            return None
            
        try:
            if self.connection_type == "v2.x":
                query = f'''
                from(bucket: "{self.config['bucket']}")
                    |> range(start: -1h)
                    |> filter(fn: (r) => r._measurement == "robot_state")
                    |> filter(fn: (r) => r.robot_id == "{robot_id}")
                    |> last()
                    |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
                '''
                result = self.query_api.query(org=self.config['org'], query=query)
                
                for table in result:
                    for record in table.records:
                        return RobotState(
                            x=float(record.values.get("x", 0)),
                            y=float(record.values.get("y", 0)),
                            angle=float(record.values.get("angle", 0)),
                            mode=record.values.get("mode", "unknown"),
                            timestamp=record.get_time()
                        )
                        
            elif self.connection_type == "v1.x":
                query = f"SELECT * FROM robot_state WHERE robot_id = '{robot_id}' ORDER BY time DESC LIMIT 1"
                result = self.client_v1.query(query)
                
                if result.raw and 'series' in result.raw and result.raw['series']:
                    series = result.raw['series'][0]
                    columns = series['columns']
                    values = series['values'][0]
                    data = dict(zip(columns, values))
                    
                    return RobotState(
                        x=float(data.get('x', 0)),
                        y=float(data.get('y', 0)),
                        angle=float(data.get('angle', 0)),
                        mode=data.get('mode', 'unknown'),
                        timestamp=datetime.fromisoformat(data['time'].replace('Z', '+00:00'))
                    )
                    
        except Exception as e:
            logger.error(f"Błąd pobierania stanu robota: {e}")
        
        return None
    
    def get_terrain_points(self, robot_id: str, last_minutes: int = 60) -> List[TerrainPoint]:
        """Pobierz punkty terenu z ostatnich X minut"""
        if not self.connected:
            return []
            
        try:
            points = []
            
            if self.connection_type == "v2.x":
                query = f'''
                from(bucket: "{self.config['bucket']}")
                    |> range(start: -{last_minutes}m)
                    |> filter(fn: (r) => r._measurement == "terrain_data")
                    |> filter(fn: (r) => r.robot_id == "{robot_id}")
                    |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
                '''
                result = self.query_api.query(org=self.config['org'], query=query)
                
                for table in result:
                    for record in table.records:
                        points.append(TerrainPoint(
                            x=int(record.values.get("x", 0)),
                            y=int(record.values.get("y", 0)),
                            distance=float(record.values.get("distance", 0)),
                            confidence=float(record.values.get("confidence", 0)),
                            is_obstacle=str(record.values.get("is_obstacle", "False")) == "True",
                            timestamp=record.get_time()
                        ))
                        
            elif self.connection_type == "v1.x":
                query = f"SELECT * FROM terrain_data WHERE robot_id = '{robot_id}' AND time > now() - {last_minutes}m"
                result = self.client_v1.query(query)
                
                if result.raw and 'series' in result.raw:
                    for series in result.raw['series']:
                        columns = series['columns']
                        for values in series['values']:
                            data = dict(zip(columns, values))
                            points.append(TerrainPoint(
                                x=int(data.get('x', 0)),
                                y=int(data.get('y', 0)),
                                distance=float(data.get('distance', 0)),
                                confidence=float(data.get('confidence', 0)),
                                is_obstacle=str(data.get('is_obstacle', 'False')) == 'True',
                                timestamp=datetime.fromisoformat(data['time'].replace('Z', '+00:00'))
                            ))
            
            return points
            
        except Exception as e:
            logger.error(f"Błąd pobierania punktów terenu: {e}")
            return []
    
    def get_exploration_stats(self, robot_id: str) -> Optional[ExplorationStats]:
        """Pobierz najnowsze statystyki eksploracji"""
        if not self.connected:
            return None
            
        try:
            if self.connection_type == "v2.x":
                query = f'''
                from(bucket: "{self.config['bucket']}")
                    |> range(start: -1h)
                    |> filter(fn: (r) => r._measurement == "exploration_stats")
                    |> filter(fn: (r) => r.robot_id == "{robot_id}")
                    |> last()
                    |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
                '''
                result = self.query_api.query(org=self.config['org'], query=query)
                
                for table in result:
                    for record in table.records:
                        return ExplorationStats(
                            coverage_percent=float(record.values.get("coverage_percent", 0)),
                            distance_traveled=float(record.values.get("distance_traveled", 0)),
                            obstacles_detected=int(record.values.get("obstacles_detected", 0)),
                            session_time_minutes=float(record.values.get("session_time_minutes", 0)),
                            exploration_efficiency=float(record.values.get("exploration_efficiency", 0)),
                            frontiers_count=0  # Nie zapisujemy tego w DB
                        )
                        
            elif self.connection_type == "v1.x":
                query = f"SELECT * FROM exploration_stats WHERE robot_id = '{robot_id}' ORDER BY time DESC LIMIT 1"
                result = self.client_v1.query(query)
                
                if result.raw and 'series' in result.raw and result.raw['series']:
                    series = result.raw['series'][0]
                    columns = series['columns']
                    values = series['values'][0]
                    data = dict(zip(columns, values))
                    
                    return ExplorationStats(
                        coverage_percent=float(data.get('coverage_percent', 0)),
                        distance_traveled=float(data.get('distance_traveled', 0)),
                        obstacles_detected=int(data.get('obstacles_detected', 0)),
                        session_time_minutes=float(data.get('session_time_minutes', 0)),
                        exploration_efficiency=float(data.get('exploration_efficiency', 0)),
                        frontiers_count=0
                    )
                    
        except Exception as e:
            logger.error(f"Błąd pobierania statystyk: {e}")
        
        return None
    
    def close(self):
        """Zamknij połączenie"""
        if self.client:
            self.client.close()
        logger.info("Połączenie z InfluxDB zamknięte")

# === ROBOT VIEWER GUI ===
class RobotViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Robot Mapping Live Viewer")
        self.root.geometry("1400x800")
        
        # Konfiguracja
        self.config = DEFAULT_CONFIG.copy()
        self.db_client = None
        self.connected = False
        self.auto_refresh = False
        self.refresh_job = None
        
        # Dane
        self.robot_state = None
        self.terrain_points = []
        self.exploration_stats = None
        self.terrain_map = np.full((100, 100), -1, dtype=float)
        self.robot_path = []
        
        # Setup GUI
        self.setup_gui()
        
        # Style
        self.setup_styles()
        
        logger.info("Robot Viewer uruchomiony")
    
    def setup_styles(self):
        """Konfiguracja stylów"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Custom styles
        style.configure('Success.TLabel', foreground='green', font=('Arial', 10, 'bold'))
        style.configure('Error.TLabel', foreground='red', font=('Arial', 10, 'bold'))
        style.configure('Title.TLabel', font=('Arial', 14, 'bold'))
        style.configure('Stat.TLabel', font=('Arial', 12, 'bold'))
    
    def setup_gui(self):
        """Konfiguracja interfejsu"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # === CONNECTION PANEL ===
        self.setup_connection_panel(main_frame)
        
        # === MAP PANEL ===
        self.setup_map_panel(main_frame)
        
        # === STATS PANEL ===
        self.setup_stats_panel(main_frame)
        
        # === LOG PANEL ===
        self.setup_log_panel(main_frame)
    
    def setup_connection_panel(self, parent):
        """Panel połączenia"""
        conn_frame = ttk.LabelFrame(parent, text="🔗 Connection Settings", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Robot IP
        ttk.Label(conn_frame, text="Robot IP:").grid(row=0, column=0, sticky=tk.W)
        self.robot_ip_var = tk.StringVar(value=self.config['robot_ip'])
        ttk.Entry(conn_frame, textvariable=self.robot_ip_var, width=20).grid(row=0, column=1, padx=(5, 20))
        
        # InfluxDB URL
        ttk.Label(conn_frame, text="InfluxDB URL:").grid(row=0, column=2, sticky=tk.W)
        self.influx_url_var = tk.StringVar(value=self.config['influxdb_url'])
        ttk.Entry(conn_frame, textvariable=self.influx_url_var, width=25).grid(row=0, column=3, padx=(5, 20))
        
        # Credentials
        ttk.Label(conn_frame, text="Username:").grid(row=1, column=0, sticky=tk.W)
        self.username_var = tk.StringVar(value=self.config['username'])
        ttk.Entry(conn_frame, textvariable=self.username_var, width=15).grid(row=1, column=1, padx=(5, 20))
        
        ttk.Label(conn_frame, text="Password:").grid(row=1, column=2, sticky=tk.W)
        self.password_var = tk.StringVar(value=self.config['password'])
        ttk.Entry(conn_frame, textvariable=self.password_var, show="*", width=15).grid(row=1, column=3, padx=(5, 20))
        
        # Robot ID
        ttk.Label(conn_frame, text="Robot ID:").grid(row=1, column=4, sticky=tk.W)
        self.robot_id_var = tk.StringVar(value=self.config['robot_id'])
        ttk.Entry(conn_frame, textvariable=self.robot_id_var, width=15).grid(row=1, column=5, padx=(5, 20))
        
        # Buttons
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_to_influx)
        self.connect_btn.grid(row=0, column=6, padx=(10, 5))
        
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_from_influx, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=7, padx=5)
        
        self.refresh_btn = ttk.Button(conn_frame, text="🔄 Refresh", command=self.manual_refresh)
        self.refresh_btn.grid(row=1, column=6, padx=(10, 5))
        
        # Auto refresh toggle
        self.auto_refresh_var = tk.BooleanVar()
        self.auto_refresh_cb = ttk.Checkbutton(conn_frame, text="Auto Refresh", 
                                             variable=self.auto_refresh_var, 
                                             command=self.toggle_auto_refresh)
        self.auto_refresh_cb.grid(row=1, column=7, padx=5)
        
        # Status
        self.status_label = ttk.Label(conn_frame, text="Disconnected", style='Error.TLabel')
        self.status_label.grid(row=2, column=0, columnspan=8, pady=(10, 0))
    
    def setup_map_panel(self, parent):
        """Panel mapy"""
        map_frame = ttk.LabelFrame(parent, text="🗺️ Live Terrain Map", padding="10")
        map_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        map_frame.columnconfigure(0, weight=1)
        map_frame.rowconfigure(0, weight=1)
        
        # Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_xlabel('X Position (dm)')
        self.ax.set_ylabel('Y Position (dm)')
        self.ax.set_title('Robot Terrain Map')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, map_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Map controls
        controls_frame = ttk.Frame(map_frame)
        controls_frame.grid(row=1, column=0, pady=(10, 0))
        
        ttk.Button(controls_frame, text="🎯 Center on Robot", command=self.center_on_robot).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_frame, text="🔍 Zoom Reset", command=self.reset_zoom).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_frame, text="💾 Save Map", command=self.save_map).pack(side=tk.LEFT, padx=5)
    
    def setup_stats_panel(self, parent):
        """Panel statystyk"""
        stats_frame = ttk.LabelFrame(parent, text="📊 Robot Statistics", padding="10")
        stats_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Robot Info
        robot_info_frame = ttk.LabelFrame(stats_frame, text="Robot Status", padding="10")
        robot_info_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.robot_pos_label = ttk.Label(robot_info_frame, text="Position: Unknown")
        self.robot_pos_label.pack(anchor=tk.W)
        
        self.robot_angle_label = ttk.Label(robot_info_frame, text="Angle: Unknown")
        self.robot_angle_label.pack(anchor=tk.W)
        
        self.robot_mode_label = ttk.Label(robot_info_frame, text="Mode: Unknown")
        self.robot_mode_label.pack(anchor=tk.W)
        
        self.last_update_label = ttk.Label(robot_info_frame, text="Last Update: Never")
        self.last_update_label.pack(anchor=tk.W)
        
        # Exploration Stats
        exploration_frame = ttk.LabelFrame(stats_frame, text="Exploration Progress", padding="10")
        exploration_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Stats grid
        stats_grid = ttk.Frame(exploration_frame)
        stats_grid.pack(fill=tk.X)
        
        # Coverage
        ttk.Label(stats_grid, text="Coverage:").grid(row=0, column=0, sticky=tk.W)
        self.coverage_label = ttk.Label(stats_grid, text="0%", style='Stat.TLabel')
        self.coverage_label.grid(row=0, column=1, sticky=tk.E)
        
        # Distance
        ttk.Label(stats_grid, text="Distance:").grid(row=1, column=0, sticky=tk.W)
        self.distance_label = ttk.Label(stats_grid, text="0.0 m", style='Stat.TLabel')
        self.distance_label.grid(row=1, column=1, sticky=tk.E)
        
        # Obstacles
        ttk.Label(stats_grid, text="Obstacles:").grid(row=2, column=0, sticky=tk.W)
        self.obstacles_label = ttk.Label(stats_grid, text="0", style='Stat.TLabel')
        self.obstacles_label.grid(row=2, column=1, sticky=tk.E)
        
        # Session Time
        ttk.Label(stats_grid, text="Session Time:").grid(row=3, column=0, sticky=tk.W)
        self.session_time_label = ttk.Label(stats_grid, text="0 min", style='Stat.TLabel')
        self.session_time_label.grid(row=3, column=1, sticky=tk.E)
        
        # Efficiency
        ttk.Label(stats_grid, text="Efficiency:").grid(row=4, column=0, sticky=tk.W)
        self.efficiency_label = ttk.Label(stats_grid, text="0 pts/min", style='Stat.TLabel')
        self.efficiency_label.grid(row=4, column=1, sticky=tk.E)
        
        # Configure grid
        stats_grid.columnconfigure(1, weight=1)
        
        # Connection Stats
        conn_stats_frame = ttk.LabelFrame(stats_frame, text="Connection Info", padding="10")
        conn_stats_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.db_type_label = ttk.Label(conn_stats_frame, text="Database: Not connected")
        self.db_type_label.pack(anchor=tk.W)
        
        self.data_points_label = ttk.Label(conn_stats_frame, text="Terrain Points: 0")
        self.data_points_label.pack(anchor=tk.W)
        
        self.refresh_rate_label = ttk.Label(conn_stats_frame, text="Refresh Rate: Manual")
        self.refresh_rate_label.pack(anchor=tk.W)
    
    def setup_log_panel(self, parent):
        """Panel logów"""
        log_frame = ttk.LabelFrame(parent, text="📋 Activity Log", padding="10")
        log_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Log text widget
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Clear button
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, pady=(5, 0))
        
        # Add initial log
        self.add_log("System initialized. Configure connection and click Connect.", "INFO")
    
    def add_log(self, message: str, level: str = "INFO"):
        """Dodaj wpis do logu"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
        # Also log to console
        if level == "ERROR":
            logger.error(message)
        elif level == "WARNING":
            logger.warning(message)
        else:
            logger.info(message)
    
    def clear_log(self):
        """Wyczyść log"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def update_config(self):
        """Aktualizuj konfigurację z GUI"""
        self.config.update({
            'robot_ip': self.robot_ip_var.get(),
            'influxdb_url': self.influx_url_var.get(),
            'username': self.username_var.get(),
            'password': self.password_var.get(),
            'robot_id': self.robot_id_var.get()
        })
    
    def connect_to_influx(self):
        """Połącz z InfluxDB"""
        self.update_config()
        
        self.add_log(f"Connecting to InfluxDB at {self.config['influxdb_url']}...")
        
        # Create client
        self.db_client = InfluxDBClient(self.config)
        
        # Connect in thread to avoid GUI freeze
        def connect_thread():
            if self.db_client.connect():
                self.root.after(0, self.on_connect_success)
            else:
                self.root.after(0, self.on_connect_error)
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connect_success(self):
        """Callback po udanym połączeniu"""
        self.connected = True
        self.status_label.config(text=f"Connected to InfluxDB ({self.db_client.connection_type})", style='Success.TLabel')
        self.connect_btn.config(state=tk.DISABLED)
        self.disconnect_btn.config(state=tk.NORMAL)
        
        self.db_type_label.config(text=f"Database: InfluxDB {self.db_client.connection_type}")
        
        self.add_log(f"Successfully connected to InfluxDB {self.db_client.connection_type}", "SUCCESS")
        
        # Immediate refresh
        self.manual_refresh()
    
    def on_connect_error(self):
        """Callback po błędzie połączenia"""
        self.connected = False
        self.status_label.config(text="Connection failed", style='Error.TLabel')
        self.add_log("Failed to connect to InfluxDB. Check settings and try again.", "ERROR")
    
    def disconnect_from_influx(self):
        """Rozłącz z InfluxDB"""
        if self.db_client:
            self.db_client.close()
            self.db_client = None
        
        self.connected = False
        self.auto_refresh_var.set(False)
        self.toggle_auto_refresh()
        
        self.status_label.config(text="Disconnected", style='Error.TLabel')
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        
        self.db_type_label.config(text="Database: Not connected")
        
        self.add_log("Disconnected from InfluxDB", "INFO")
    
    def toggle_auto_refresh(self):
        """Przełącz auto-refresh"""
        self.auto_refresh = self.auto_refresh_var.get()
        
        if self.auto_refresh and self.connected:
            self.refresh_rate_label.config(text=f"Refresh Rate: {self.config['refresh_interval']/1000:.1f}s")
            self.schedule_refresh()
            self.add_log("Auto-refresh enabled", "INFO")
        else:
            self.refresh_rate_label.config(text="Refresh Rate: Manual")
            if self.refresh_job:
                self.root.after_cancel(self.refresh_job)
                self.refresh_job = None
            self.add_log("Auto-refresh disabled", "INFO")
    
    def schedule_refresh(self):
        """Zaplanuj następny refresh"""
        if self.auto_refresh and self.connected:
            self.refresh_job = self.root.after(self.config['refresh_interval'], self.auto_refresh_callback)
    
    def auto_refresh_callback(self):
        """Callback dla auto-refresh"""
        self.manual_refresh()
        self.schedule_refresh()
    
    def manual_refresh(self):
        """Ręczny refresh danych"""
        if not self.connected or not self.db_client:
            self.add_log("Not connected to database", "WARNING")
            return
        
        def refresh_thread():
            try:
                # Pobierz dane
                robot_state = self.db_client.get_latest_robot_state(self.config['robot_id'])
                terrain_points = self.db_client.get_terrain_points(self.config['robot_id'], 60)
                exploration_stats = self.db_client.get_exploration_stats(self.config['robot_id'])
                
                # Update GUI w main thread
                self.root.after(0, lambda: self.update_data(robot_state, terrain_points, exploration_stats))
                
            except Exception as e:
                self.root.after(0, lambda: self.add_log(f"Refresh error: {e}", "ERROR"))
        
        threading.Thread(target=refresh_thread, daemon=True).start()
    
    def update_data(self, robot_state: Optional[RobotState], 
                   terrain_points: List[TerrainPoint], 
                   exploration_stats: Optional[ExplorationStats]):
        """Aktualizuj dane w GUI"""
        
        # Update robot state
        if robot_state:
            self.robot_state = robot_state
            self.robot_pos_label.config(text=f"Position: ({robot_state.x:.1f}, {robot_state.y:.1f})")
            self.robot_angle_label.config(text=f"Angle: {robot_state.angle:.1f}°")
            self.robot_mode_label.config(text=f"Mode: {robot_state.mode}")
            self.last_update_label.config(text=f"Last Update: {robot_state.timestamp.strftime('%H:%M:%S')}")
        
        # Update terrain points
        self.terrain_points = terrain_points
        self.data_points_label.config(text=f"Terrain Points: {len(terrain_points)}")
        
        # Update exploration stats
        if exploration_stats:
            self.exploration_stats = exploration_stats
            self.coverage_label.config(text=f"{exploration_stats.coverage_percent:.1f}%")
            self.distance_label.config(text=f"{exploration_stats.distance_traveled:.1f} m")
            self.obstacles_label.config(text=str(exploration_stats.obstacles_detected))
            self.session_time_label.config(text=f"{exploration_stats.session_time_minutes:.1f} min")
            self.efficiency_label.config(text=f"{exploration_stats.exploration_efficiency:.1f} pts/min")
        
        # Update map
        self.update_map()
        
        self.add_log(f"Data refreshed - {len(terrain_points)} terrain points", "INFO")
    
    def update_map(self):
        """Aktualizuj mapę"""
        self.ax.clear()
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_xlabel('X Position (dm)')
        self.ax.set_ylabel('Y Position (dm)')
        self.ax.set_title('Robot Terrain Map - Live View')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Reset terrain map
        self.terrain_map.fill(-1)
        
        # Plot terrain points
        if self.terrain_points:
            for point in self.terrain_points:
                if 0 <= point.x < 100 and 0 <= point.y < 100:
                    self.terrain_map[point.y, point.x] = point.distance
            
            # Create color map
            x_coords, y_coords, colors = [], [], []
            for point in self.terrain_points:
                if 0 <= point.x < 100 and 0 <= point.y < 100:
                    x_coords.append(point.x)
                    y_coords.append(point.y)
                    
                    # Color based on distance
                    if point.distance < 100:
                        colors.append('blue')    # Close
                    elif point.distance < 200:
                        colors.append('green')   # Medium
                    else:
                        colors.append('yellow')  # Far
            
            if x_coords:
                self.ax.scatter(x_coords, y_coords, c=colors, s=4, alpha=0.7)
        
        # Plot robot
        if self.robot_state:
            x, y = self.robot_state.x, self.robot_state.y
            angle = np.radians(self.robot_state.angle)
            
            # Robot body
            self.ax.plot(x, y, 'ro', markersize=10, label='Robot')
            
            # Direction arrow
            dx = 3 * np.sin(angle)
            dy = -3 * np.cos(angle)  # Negative because Y is flipped
            self.ax.arrow(x, y, dx, dy, head_width=1, head_length=1, fc='red', ec='red')
        
        # Add legend
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Close (0-1m)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label='Medium (1-2m)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', markersize=8, label='Far (2m+)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Robot')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')
        
        self.canvas.draw()
    
    def center_on_robot(self):
        """Wycentruj mapę na robocie"""
        if self.robot_state:
            x, y = self.robot_state.x, self.robot_state.y
            margin = 20
            self.ax.set_xlim(max(0, x - margin), min(100, x + margin))
            self.ax.set_ylim(max(0, y - margin), min(100, y + margin))
            self.canvas.draw()
            self.add_log("Map centered on robot", "INFO")
    
    def reset_zoom(self):
        """Resetuj zoom mapy"""
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.canvas.draw()
        self.add_log("Map zoom reset", "INFO")
    
    def save_map(self):
        """Zapisz mapę do pliku"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_map_{timestamp}.png"
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.add_log(f"Map saved as {filename}", "SUCCESS")
            messagebox.showinfo("Success", f"Map saved as {filename}")
        except Exception as e:
            self.add_log(f"Error saving map: {e}", "ERROR")
            messagebox.showerror("Error", f"Failed to save map: {e}")

def main():
    """Główna funkcja"""
    # Check dependencies
    missing_deps = []
    try:
        import tkinter
    except ImportError:
        missing_deps.append("tkinter")
    
    try:
        import matplotlib
    except ImportError:
        missing_deps.append("matplotlib")
    
    try:
        import numpy
    except ImportError:
        missing_deps.append("numpy")
    
    try:
        import requests
    except ImportError:
        missing_deps.append("requests")
    
    try:
        import influxdb_client
    except ImportError:
        try:
            import influxdb
        except ImportError:
            missing_deps.append("influxdb-client or influxdb")
    
    if missing_deps:
        print("❌ Missing dependencies:")
        for dep in missing_deps:
            print(f"   pip install {dep}")
        print("\nPlease install missing dependencies and try again.")
        return
    
    # Create and run GUI
    root = tk.Tk()
    app = RobotViewer(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n👋 Robot Viewer closed by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if app.db_client:
            app.db_client.close()

if __name__ == "__main__":
    main()

