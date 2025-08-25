from flask import Flask, Response, request, jsonify, render_template_string
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import RPi.GPIO as GPIO
import ArducamDepthCamera as ac
import logging
import time
import signal
import sys
import smbus2
from math import floor, cos, sin, radians, sqrt
import psutil
import atexit
from waitress import serve
import threading
import json
from collections import deque
from datetime import datetime, timezone
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any

# === NOWE: InfluxDB Integration ===
try:
    from influxdb_client import InfluxDBClient, Point, WritePrecision
    from influxdb_client.client.write_api import SYNCHRONOUS, ASYNCHRONOUS
    from influxdb_client.client.query_api import FluxRecord
    INFLUXDB_AVAILABLE = True
    print("✅ InfluxDB client loaded successfully")
except ImportError:
    INFLUXDB_AVAILABLE = False
    print("❌ InfluxDB client not available. Install: pip install influxdb-client")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Konfiguracja logowania
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# === KONFIGURACJA INFLUXDB ===
INFLUXDB_CONFIG = {
    "url": "http://127.0.0.1:8086",
    "username": "pi",
    "password": "raspberry", 
    "org": "mapper",
    "bucket": "mapper",
    "robot_id": "robot_001"
}

# Konfiguracja ToF
MAX_DISTANCE = 4000
CONFIDENCE_VALUE = 50
OBSTACLE_THRESHOLD = 700  # mm
CRITICAL_DISTANCE = 100   # mm - bardzo bliska przeszkoda
JPEG_QUALITY = 80

# Inicjalizacja kamery ToF
cam_tof = ac.ArducamCamera()
ret = cam_tof.open(ac.Connection.CSI, 0)
if ret != 0:
    logger.error(f"Nie udało się otworzyć kamery ToF. Kod błędu: {ret}")
    exit(0)
ret = cam_tof.start(ac.FrameType.DEPTH)
if ret != 0:
    logger.error(f"Nie udało się uruchomić kamery ToF (DEPTH). Kod błędu: {ret}")
    cam_tof.close()
    exit(0)
cam_tof.setControl(ac.Control.RANGE, MAX_DISTANCE)

current_confidence = CONFIDENCE_VALUE

# Konfiguracja GPIO dla silników
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
MOTOR1_IN1 = 4  # Lewy sterownik
MOTOR1_IN2 = 17
MOTOR1_IN3 = 27
MOTOR1_IN4 = 22
MOTOR2_IN1 = 18  # Prawy sterownik
MOTOR2_IN2 = 23
MOTOR2_IN3 = 24
MOTOR2_IN4 = 25

# Inicjalizacja GPIO
if not hasattr(app, 'gpio_initialized'):
    GPIO.setup([MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4], GPIO.OUT)
    app.gpio_initialized = True

# Zmienne dla trybu automatycznego
auto_mode = False
auto_thread = None
stop_auto = False

# Mapa terenu - siatka 100x100 punktów (każdy punkt = 10cm x 10cm w rzeczywistości)
MAP_SIZE = 100
terrain_map = np.full((MAP_SIZE, MAP_SIZE), -1, dtype=np.float32)  # -1 = niezbadane
robot_x = MAP_SIZE // 2  # Pozycja robota na mapie (środek)
robot_y = MAP_SIZE // 2
robot_angle = 0  # Kąt robota w stopniach (0 = północ)

# Historia pozycji robota
robot_path = deque(maxlen=1000)

# === NOWE: Statystyki eksploracji ===
session_start_time = time.time()
total_distance_traveled = 0
obstacles_detected = 0
total_exploration_points = 0

# === NOWE: InfluxDB Data Classes ===
@dataclass
class RobotState:
    timestamp: datetime
    robot_id: str
    x: float
    y: float
    angle: float
    mode: str  # 'manual', 'auto'
    battery_level: Optional[float] = None
    speed: Optional[float] = None

@dataclass
class TerrainPoint:
    timestamp: datetime
    robot_id: str
    x: int
    y: int
    distance: float
    confidence: float
    is_obstacle: bool

@dataclass
class ExplorationStats:
    timestamp: datetime
    robot_id: str
    coverage_percent: float
    distance_traveled: float
    obstacles_detected: int
    session_time_minutes: float
    exploration_efficiency: float

# === NOWE: InfluxDB Manager ===
class InfluxDBManager:
    def __init__(self, config):
        self.config = config
        self.client = None
        self.write_api = None
        self.query_api = None
        self.connected = False
        self.last_terrain_save = 0
        self.connect()
        
    def connect(self):
        """Nawiąż połączenie z InfluxDB"""
        if not INFLUXDB_AVAILABLE:
            logger.warning("InfluxDB client not available")
            return False
            
        try:
            # Utwórz token z username/password (dla InfluxDB 2.x)
            # Dla InfluxDB 1.x użyj: InfluxDBClient(host='127.0.0.1', port=8086, username='pi', password='raspberry', database='robot_mapping')
            
            # InfluxDB 2.x authentication
            auth_token = f"{self.config['username']}:{self.config['password']}"
            
            self.client = InfluxDBClient(
                url=self.config['url'],
                token='90jddHVMoo6nZSgc3hJkX8ZLrCcyjoyIw67xVg2viUVdhh9XXrS-QOZX6uTbaia3A75NPRFU5Y4WFPhKvgQrLA==',  # W InfluxDB 2.x możesz też użyć właściwego tokena
                org=self.config['org']
            )
            
            self.write_api = self.client.write_api(write_options=ASYNCHRONOUS)
            self.query_api = self.client.query_api()
            
            # Test połączenia
            self.client.ping()
            self.connected = True
            logger.info("✅ Połączono z InfluxDB")
            return True
            
        except Exception as e:
            logger.error(f"❌ Błąd połączenia z InfluxDB: {e}")
            # Fallback dla InfluxDB 1.x
            try:
                from influxdb import InfluxDBClient as InfluxDBClientV1
                self.client_v1 = InfluxDBClientV1(
                    host='127.0.0.1', 
                    port=8086,
                    username=self.config['username'],
                    password=self.config['password'],
                    database='robot_mapping'
                )
                self.client_v1.create_database('robot_mapping')
                self.connected = True
                logger.info("✅ Połączono z InfluxDB v1.x")
                return True
            except Exception as e2:
                logger.error(f"❌ Błąd połączenia z InfluxDB v1.x: {e2}")
                self.connected = False
                return False
    
    def write_robot_state(self, state: RobotState):
        """Zapisz stan robota"""
        if not self.connected:
            return
            
        try:
            if hasattr(self, 'client_v1'):  # InfluxDB 1.x
                json_body = [{
                    "measurement": "robot_state",
                    "tags": {
                        "robot_id": state.robot_id,
                        "mode": state.mode
                    },
                    "time": state.timestamp.isoformat(),
                    "fields": {
                        "x": float(state.x),
                        "y": float(state.y),
                        "angle": float(state.angle)
                    }
                }]
                self.client_v1.write_points(json_body)
            else:  # InfluxDB 2.x
                point = Point("robot_state") \
                    .tag("robot_id", state.robot_id) \
                    .tag("mode", state.mode) \
                    .field("x", float(state.x)) \
                    .field("y", float(state.y)) \
                    .field("angle", float(state.angle)) \
                    .time(state.timestamp, WritePrecision.MS)
                
                if state.battery_level:
                    point = point.field("battery_level", float(state.battery_level))
                if state.speed:
                    point = point.field("speed", float(state.speed))
                    
                self.write_api.write(bucket=self.config['bucket'], org=self.config['org'], record=point)
                
        except Exception as e:
            logger.error(f"Błąd zapisu stanu robota do InfluxDB: {e}")
    
    def write_terrain_batch(self, points: List[TerrainPoint]):
        """Zapisz punkty terenu w batch (wydajniej)"""
        if not self.connected or not points:
            return
            
        try:
            if hasattr(self, 'client_v1'):  # InfluxDB 1.x
                json_body = []
                for point in points:
                    json_body.append({
                        "measurement": "terrain_data",
                        "tags": {
                            "robot_id": point.robot_id,
                            "is_obstacle": str(point.is_obstacle)
                        },
                        "time": point.timestamp.isoformat(),
                        "fields": {
                            "x": point.x,
                            "y": point.y,
                            "distance": float(point.distance),
                            "confidence": float(point.confidence)
                        }
                    })
                self.client_v1.write_points(json_body)
            else:  # InfluxDB 2.x
                influx_points = []
                for point in points:
                    p = Point("terrain_data") \
                        .tag("robot_id", point.robot_id) \
                        .tag("is_obstacle", str(point.is_obstacle)) \
                        .field("x", point.x) \
                        .field("y", point.y) \
                        .field("distance", float(point.distance)) \
                        .field("confidence", float(point.confidence)) \
                        .time(point.timestamp, WritePrecision.MS)
                    influx_points.append(p)
                
                self.write_api.write(bucket=self.config['bucket'], org=self.config['org'], record=influx_points)
                
        except Exception as e:
            logger.error(f"Błąd zapisu terenu do InfluxDB: {e}")
    
    def write_exploration_stats(self, stats: ExplorationStats):
        """Zapisz statystyki eksploracji"""
        if not self.connected:
            return
            
        try:
            if hasattr(self, 'client_v1'):  # InfluxDB 1.x
                json_body = [{
                    "measurement": "exploration_stats",
                    "tags": {
                        "robot_id": stats.robot_id
                    },
                    "time": stats.timestamp.isoformat(),
                    "fields": {
                        "coverage_percent": float(stats.coverage_percent),
                        "distance_traveled": float(stats.distance_traveled),
                        "obstacles_detected": stats.obstacles_detected,
                        "session_time_minutes": float(stats.session_time_minutes),
                        "exploration_efficiency": float(stats.exploration_efficiency)
                    }
                }]
                self.client_v1.write_points(json_body)
            else:  # InfluxDB 2.x
                point = Point("exploration_stats") \
                    .tag("robot_id", stats.robot_id) \
                    .field("coverage_percent", float(stats.coverage_percent)) \
                    .field("distance_traveled", float(stats.distance_traveled)) \
                    .field("obstacles_detected", stats.obstacles_detected) \
                    .field("session_time_minutes", float(stats.session_time_minutes)) \
                    .field("exploration_efficiency", float(stats.exploration_efficiency)) \
                    .time(stats.timestamp, WritePrecision.MS)
                
                self.write_api.write(bucket=self.config['bucket'], org=self.config['org'], record=point)
                
        except Exception as e:
            logger.error(f"Błąd zapisu statystyk do InfluxDB: {e}")
    
    def get_latest_robot_state(self, robot_id: str) -> Optional[Dict]:
        """Pobierz najnowszy stan robota"""
        if not self.connected:
            return None
            
        try:
            if hasattr(self, 'client_v1'):  # InfluxDB 1.x
                query = f"SELECT * FROM robot_state WHERE robot_id = '{robot_id}' ORDER BY time DESC LIMIT 1"
                result = self.client_v1.query(query)
                if result.raw and 'series' in result.raw:
                    return result.raw['series'][0]['values'][0] if result.raw['series'] else None
            else:  # InfluxDB 2.x
                query = f'''
                from(bucket: "{self.config['bucket']}")
                    |> range(start: -1h)
                    |> filter(fn: (r) => r._measurement == "robot_state")
                    |> filter(fn: (r) => r.robot_id == "{robot_id}")
                    |> last()
                '''
                result = self.query_api.query(org=self.config['org'], query=query)
                
                if result:
                    state_data = {}
                    for table in result:
                        for record in table.records:
                            state_data[record.get_field()] = record.get_value()
                            state_data['time'] = record.get_time()
                    return state_data if state_data else None
                    
        except Exception as e:
            logger.error(f"Błąd pobierania stanu robota z InfluxDB: {e}")
        
        return None
    
    def get_terrain_map_live(self, robot_id: str, last_minutes: int = 60) -> List[Dict]:
        """Pobierz punkty terenu z ostatnich X minut"""
        if not self.connected:
            return []
            
        try:
            if hasattr(self, 'client_v1'):  # InfluxDB 1.x
                query = f"SELECT * FROM terrain_data WHERE robot_id = '{robot_id}' AND time > now() - {last_minutes}m"
                result = self.client_v1.query(query)
                points = []
                if result.raw and 'series' in result.raw:
                    for series in result.raw['series']:
                        columns = series['columns']
                        for values in series['values']:
                            point = dict(zip(columns, values))
                            points.append(point)
                return points
            else:  # InfluxDB 2.x
                query = f'''
                from(bucket: "{self.config['bucket']}")
                    |> range(start: -{last_minutes}m)
                    |> filter(fn: (r) => r._measurement == "terrain_data")
                    |> filter(fn: (r) => r.robot_id == "{robot_id}")
                    |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
                '''
                result = self.query_api.query(org=self.config['org'], query=query)
                
                points = []
                for table in result:
                    for record in table.records:
                        point = {
                            "time": record.get_time(),
                            "x": record.values.get("x"),
                            "y": record.values.get("y"), 
                            "distance": record.values.get("distance"),
                            "confidence": record.values.get("confidence"),
                            "is_obstacle": record.values.get("is_obstacle") == "True"
                        }
                        points.append(point)
                return points
                
        except Exception as e:
            logger.error(f"Błąd pobierania mapy terenu z InfluxDB: {e}")
        
        return []
    
    def close(self):
        """Zamknij połączenie"""
        if self.client:
            self.client.close()
            logger.info("Połączenie z InfluxDB zamknięte")

# Inicjalizacja InfluxDB Manager
influx_manager = InfluxDBManager(INFLUXDB_CONFIG)

# === NOWE: Inteligentna eksploracja ===
class SmartExploration:
    def __init__(self):
        self.frontiers = []
        self.explored_areas = set()
        self.current_target = None
        
    def find_frontiers(self, terrain_map):
        """Znajdź granice między zbadanymi a niezbadanymi obszarami"""
        frontiers = []
        for y in range(1, MAP_SIZE-1):
            for x in range(1, MAP_SIZE-1):
                if terrain_map[y,x] == -1:  # Niezbadane
                    # Sprawdź czy ma sąsiadów zbadanych
                    neighbors = [terrain_map[y+dy, x+dx] 
                               for dy in [-1,0,1] for dx in [-1,0,1] if (dy != 0 or dx != 0)]
                    if any(n >= 0 for n in neighbors):
                        frontiers.append((x, y))
        
        # Grupuj frontiersy w klastry
        return self.cluster_frontiers(frontiers)
    
    def cluster_frontiers(self, frontiers):
        """Grupuj frontiersy w klastry aby uniknąć małych odizolowanych punktów"""
        if not frontiers:
            return []
        
        clusters = []
        for frontier in frontiers:
            added_to_cluster = False
            for cluster in clusters:
                # Sprawdź czy frontier jest blisko istniejącego klastra
                for point in cluster:
                    if abs(frontier[0] - point[0]) <= 2 and abs(frontier[1] - point[1]) <= 2:
                        cluster.append(frontier)
                        added_to_cluster = True
                        break
                if added_to_cluster:
                    break
            
            if not added_to_cluster:
                clusters.append([frontier])
        
        # Zwróć środek każdego klastra o odpowiedniej wielkości
        cluster_centers = []
        for cluster in clusters:
            if len(cluster) >= 3:  # Minimum 3 punkty w klastrze
                center_x = sum(p[0] for p in cluster) // len(cluster)
                center_y = sum(p[1] for p in cluster) // len(cluster)
                cluster_centers.append((center_x, center_y))
        
        return cluster_centers
    
    def choose_next_target(self, robot_pos, frontiers):
        """Wybierz najbliższą granicę do eksploracji"""
        if not frontiers:
            return None
        
        def distance(pos1, pos2):
            return ((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)**0.5
        
        return min(frontiers, key=lambda f: distance(robot_pos, f))

# Instancja klasy inteligentnej eksploracji
smart_exploration = SmartExploration()

# Funkcje sterowania silnikami
def stop():
    GPIO.output([MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4], GPIO.LOW)
    logger.info("Jazda: zatrzymano")

def forward():
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    GPIO.output(MOTOR1_IN3, GPIO.HIGH)
    GPIO.output(MOTOR1_IN4, GPIO.LOW)
    GPIO.output(MOTOR2_IN1, GPIO.LOW)
    GPIO.output(MOTOR2_IN2, GPIO.HIGH)
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    logger.info("Jazda: do przodu")

def backward():
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.HIGH)
    GPIO.output(MOTOR1_IN3, GPIO.LOW)
    GPIO.output(MOTOR1_IN4, GPIO.HIGH)
    GPIO.output(MOTOR2_IN1, GPIO.HIGH)
    GPIO.output(MOTOR2_IN2, GPIO.LOW)
    GPIO.output(MOTOR2_IN3, GPIO.HIGH)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    logger.info("Jazda: do tyłu")

def left():
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.HIGH)
    GPIO.output(MOTOR1_IN3, GPIO.LOW)
    GPIO.output(MOTOR1_IN4, GPIO.HIGH)
    GPIO.output(MOTOR2_IN1, GPIO.LOW)
    GPIO.output(MOTOR2_IN2, GPIO.HIGH)
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    logger.info("Jazda: w lewo")

def right():
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    GPIO.output(MOTOR1_IN3, GPIO.HIGH)
    GPIO.output(MOTOR1_IN4, GPIO.LOW)
    GPIO.output(MOTOR2_IN1, GPIO.HIGH)
    GPIO.output(MOTOR2_IN2, GPIO.LOW)
    GPIO.output(MOTOR2_IN3, GPIO.HIGH)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    logger.info("Jazda: w prawo")

# PCA9685 dla serw
class PCA9685:
    def __init__(self, address=0x40, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self.writeReg(0x00, 0x00)
        self.setPWMFreq(50)
        self.servo_angles = {0: 90, 1: 90}

    def writeReg(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def setPWMFreq(self, freq):
        prescale = floor((25000000 / (4096 * freq)) + 0.5) - 1
        oldmode = self.bus.read_byte_data(self.address, 0x00)
        newmode = (oldmode & 0x7F) | 0x10
        self.writeReg(0x00, newmode)
        self.writeReg(0xFE, prescale)
        self.writeReg(0x00, oldmode)
        time.sleep(0.005)
        self.writeReg(0x00, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        self.writeReg(0x06 + 4 * channel, on & 0xFF)
        self.writeReg(0x07 + 4 * channel, on >> 8)
        self.writeReg(0x08 + 4 * channel, off & 0xFF)
        self.writeReg(0x09 + 4 * channel, off >> 8)

    def setServoAngle(self, channel, angle):
        angle = max(10, min(170, angle))
        pulse_width_ms = 0.5 + (angle / 180.0) * 2.0
        pwm_value = floor((pulse_width_ms * 4096 / 20) + 0.5)
        self.setPWM(channel, 0, pwm_value)
        self.servo_angles[channel] = angle
        logger.info(f"Ustawiono serwo na kanale {channel}: {angle}°")

# Inicjalizacja PCA9685
try:
    pwm = PCA9685()
except Exception as e:
    logger.error(f"Błąd PCA9685: {e}")
    cam_tof.stop()
    cam_tof.close()
    stop()
    GPIO.cleanup()
    exit(1)

# === NOWE: Funkcje zapisu do bazy danych ===
def save_robot_state_to_db():
    """Zapisz aktualny stan robota do InfluxDB"""
    try:
        state = RobotState(
            timestamp=datetime.now(timezone.utc),
            robot_id=INFLUXDB_CONFIG['robot_id'],
            x=float(robot_x),
            y=float(robot_y),
            angle=float(robot_angle),
            mode="auto" if auto_mode else "manual"
        )
        influx_manager.write_robot_state(state)
    except Exception as e:
        logger.error(f"Błąd zapisu stanu robota do DB: {e}")

def save_terrain_data_to_db():
    """Zapisz dane terenu do InfluxDB (batch)"""
    try:
        points = []
        timestamp = datetime.now(timezone.utc)
        
        # Konwertuj tylko nowo odkryte punkty (dla wydajności)
        for y in range(MAP_SIZE):
            for x in range(MAP_SIZE):
                if terrain_map[y, x] >= 0:  # Tylko zbadane punkty
                    distance = float(terrain_map[y, x])
                    
                    point = TerrainPoint(
                        timestamp=timestamp,
                        robot_id=INFLUXDB_CONFIG['robot_id'],
                        x=x, y=y,
                        distance=distance,
                        confidence=90.0,  # Przybliżona wartość
                        is_obstacle=distance < 70  # 70cm threshold
                    )
                    points.append(point)
        
        if points:
            # Zapisuj w partiach po 100 punktów dla wydajności
            batch_size = 100
            for i in range(0, len(points), batch_size):
                batch = points[i:i+batch_size]
                influx_manager.write_terrain_batch(batch)
            
            logger.info(f"Zapisano {len(points)} punktów terenu do DB")
            
    except Exception as e:
        logger.error(f"Błąd zapisu terenu do DB: {e}")

def save_exploration_stats_to_db():
    """Zapisz statystyki eksploracji do InfluxDB"""
    try:
        stats_data = get_exploration_statistics()
        
        stats = ExplorationStats(
            timestamp=datetime.now(timezone.utc),
            robot_id=INFLUXDB_CONFIG['robot_id'],
            coverage_percent=stats_data['coverage_percent'],
            distance_traveled=stats_data['distance_traveled_meters'],
            obstacles_detected=stats_data['obstacles_detected'],
            session_time_minutes=stats_data['session_time_minutes'],
            exploration_efficiency=stats_data['exploration_efficiency']
        )
        
        influx_manager.write_exploration_stats(stats)
        
    except Exception as e:
        logger.error(f"Błąd zapisu statystyk do DB: {e}")

# Funkcje mapowania - ZMODYFIKOWANE
def update_robot_position(direction, distance_cm=10):
    """Aktualizuje pozycję robota na mapie + zapis do DB"""
    global robot_x, robot_y, robot_angle, total_distance_traveled
    
    if direction == 'forward':
        robot_x += int(distance_cm * sin(radians(robot_angle)) / 10)
        robot_y -= int(distance_cm * cos(radians(robot_angle)) / 10)
        total_distance_traveled += distance_cm / 100  # konwersja do metrów
    elif direction == 'backward':
        robot_x -= int(distance_cm * sin(radians(robot_angle)) / 10)
        robot_y += int(distance_cm * cos(radians(robot_angle)) / 10)
        total_distance_traveled += distance_cm / 100
    elif direction == 'left':
        robot_angle = (robot_angle - 15) % 360
    elif direction == 'right':
        robot_angle = (robot_angle + 15) % 360
    
    # Ograniczenia mapy
    robot_x = max(5, min(MAP_SIZE - 5, robot_x))
    robot_y = max(5, min(MAP_SIZE - 5, robot_y))
    
    # Dodaj do ścieżki
    robot_path.append((robot_x, robot_y))
    
    # === NOWE: Zapisz stan robota do DB ===
    save_robot_state_to_db()

def get_depth_measurements():
    """Pobiera pomiary głębi z kamery ToF"""
    try:
        frame = cam_tof.requestFrame(1000)
        if frame and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data
            
            # Filtracja danych z odpowiednią ufnością
            valid_mask = confidence_buf >= current_confidence
            depth_data = np.where(valid_mask, depth_buf, 0)
            
            cam_tof.releaseFrame(frame)
            return depth_data
        return None
    except Exception as e:
        logger.error(f"Błąd pobierania danych głębi: {e}")
        return None

def update_terrain_map(depth_data):
    """Aktualizuje mapę terenu na podstawie danych z ToF + zapis do DB"""
    global total_exploration_points
    
    if depth_data is None:
        return
    
    height, width = depth_data.shape
    center_x, center_y = width // 2, height // 2
    
    points_added = 0
    # Próbkowanie punktów z obrazu głębi
    for i in range(0, height, 20):  # Co 20 pikseli
        for j in range(0, width, 20):
            if depth_data[i, j] > 0:  # Ważny pomiar
                # Konwersja współrzędnych piksela na pozycję w mapie
                angle_offset = (j - center_x) * 0.5  # Przybliżony kąt
                distance_cm = depth_data[i, j] / 10  # mm na cm
                
                # Pozycja punktu względem robota
                point_angle = robot_angle + angle_offset
                map_x = robot_x + int(distance_cm * sin(radians(point_angle)) / 10)
                map_y = robot_y - int(distance_cm * cos(radians(point_angle)) / 10)
                
                # Zapisz w mapie jeśli w granicach
                if 0 <= map_x < MAP_SIZE and 0 <= map_y < MAP_SIZE:
                    if terrain_map[map_y, map_x] == -1:  # Nowy punkt
                        points_added += 1
                    terrain_map[map_y, map_x] = min(distance_cm, 400)  # Max 4m
    
    total_exploration_points += points_added
    
    # === NOWE: Zapisz dane terenu do DB co 15 sekund ===
    if not hasattr(update_terrain_map, 'last_db_save'):
        update_terrain_map.last_db_save = 0
        
    if time.time() - update_terrain_map.last_db_save > 15:  # Co 15 sekund
        save_terrain_data_to_db()
        save_exploration_stats_to_db()  # Zapisz też statystyki
        update_terrain_map.last_db_save = time.time()

def detect_critical_obstacle(depth_data):
    """Wykrywa bardzo bliskie przeszkody"""
    global obstacles_detected
    
    if depth_data is None:
        return False
    
    height, width = depth_data.shape
    center_region = depth_data[height//3:2*height//3, width//3:2*width//3]
    
    valid_depths = center_region[center_region > 0]
    if len(valid_depths) > 0:
        min_distance = np.min(valid_depths)
        if min_distance < CRITICAL_DISTANCE:
            obstacles_detected += 1
            return True
    
    return False

def get_best_direction(depth_data):
    """Określa najlepszy kierunek jazdy na podstawie danych głębi"""
    if depth_data is None:
        return 'forward'
    
    height, width = depth_data.shape
    
    # Podziel obraz na sektory: lewy, środkowy, prawy
    left_sector = depth_data[:, :width//3]
    center_sector = depth_data[:, width//3:2*width//3]
    right_sector = depth_data[:, 2*width//3:]
    
    # Oblicz średnią odległość w każdym sektorze
    def get_avg_distance(sector):
        valid_depths = sector[sector > 0]
        return np.mean(valid_depths) if len(valid_depths) > 0 else 0
    
    left_dist = get_avg_distance(left_sector)
    center_dist = get_avg_distance(center_sector)
    right_dist = get_avg_distance(right_sector)
    
    # Wybierz kierunek z największą średnią odległością
    if center_dist > OBSTACLE_THRESHOLD:
        return 'forward'
    elif left_dist > right_dist and left_dist > OBSTACLE_THRESHOLD:
        return 'left'
    elif right_dist > OBSTACLE_THRESHOLD:
        return 'right'
    else:
        return 'backward'

def auto_explore():
    """Główna pętla automatycznego eksplorowania z inteligentną nawigacją"""
    global stop_auto
    
    logger.info("Rozpoczęto inteligentną eksplorację")
    last_direction_change = time.time()
    current_direction = 'forward'
    target_position = None
    
    while not stop_auto:
        try:
            # Pobierz dane głębi
            depth_data = get_depth_measurements()
            
            # Aktualizuj mapę
            update_terrain_map(depth_data)
            
            # Sprawdź krytyczne przeszkody
            if detect_critical_obstacle(depth_data):
                stop()
                logger.info("Auto: Wykryto krytyczną przeszkodę - STOP")
                time.sleep(0.5)
                
                # Cofnij się trochę
                backward()
                update_robot_position('backward')
                time.sleep(1)
                stop()
                
                # Zmień kierunek
                current_direction = 'right'
                last_direction_change = time.time()
                target_position = None  # Resetuj cel
            
            # === NOWE: Inteligentne planowanie eksploracji ===
            if target_position is None or time.time() - last_direction_change > 10:
                frontiers = smart_exploration.find_frontiers(terrain_map)
                if frontiers:
                    target_position = smart_exploration.choose_next_target(
                        (robot_x, robot_y), frontiers
                    )
                    logger.info(f"New exploration target: {target_position}")
                    socketio.emit('status_update', {
                        'message': f'Eksploration: goal ({target_position[0]}, {target_position[1]})'
                    })
                else:
                    # Wszystko zbadane - patrol
                    logger.info("Area fully explored: patrol")
                    socketio.emit('status_update', {'message': 'Area fully explored: patrol'})
                    target_position = None
                
                last_direction_change = time.time()
            
            # Nawiguj do celu lub kontynuuj eksplorację
            if target_position:
                # Prosta nawigacja do celu
                dx = target_position[0] - robot_x
                dy = target_position[1] - robot_y
                
                # Sprawdź czy dotarł do celu
                if abs(dx) <= 2 and abs(dy) <= 2:
                    target_position = None
                    logger.info("Dotarto do celu")
                else:
                    # Wybierz kierunek do celu
                    if abs(dx) > abs(dy):
                        current_direction = 'right' if dx > 0 else 'left'
                    else:
                        current_direction = 'forward' if dy < 0 else 'backward'
            else:
                # Jeśli brak celu, używaj starego algorytmu
                if time.time() - last_direction_change > 3:
                    new_direction = get_best_direction(depth_data)
                    if new_direction != current_direction:
                        current_direction = new_direction
                        last_direction_change = time.time()
            
            # Wykonaj ruch
            if current_direction == 'forward':
                forward()
                update_robot_position('forward')
                time.sleep(0.8)
            elif current_direction == 'left':
                left()
                update_robot_position('left')
                time.sleep(0.3)
            elif current_direction == 'right':
                right()
                update_robot_position('right')
                time.sleep(0.3)
            elif current_direction == 'backward':
                backward()
                update_robot_position('backward')
                time.sleep(0.5)
            
            stop()
            
            # Wyślij aktualizację mapy przez WebSocket
            send_map_update()
            
            time.sleep(0.5)
            
        except Exception as e:
            logger.error(f"Error in auto_explore: {e}")
            stop()
            break
    
    stop()
    logger.info("Zakończono inteligentną eksplorację")

def send_map_update():
    """Wysyła aktualizację mapy przez WebSocket"""
    try:
        # Przygotuj dane mapy do wysłania
        map_data = {
            'terrain': terrain_map.tolist(),
            'robot_pos': {'x': robot_x, 'y': robot_y, 'angle': robot_angle},
            'path': list(robot_path)[-100:],  # Ostatnie 100 punktów ścieżki
            'timestamp': time.time(),
            'statistics': get_exploration_statistics()
        }
        
        socketio.emit('map_update', map_data)
    except Exception as e:
        logger.error(f"Błąd wysyłania mapy: {e}")

# === NOWE: Funkcje statystyk ===
def get_exploration_statistics():
    """Zwraca statystyki eksploracji"""
    total_points = MAP_SIZE * MAP_SIZE
    explored_points = int(np.sum(terrain_map >= 0))
    coverage_percent = (explored_points / total_points) * 100
    
    session_time = time.time() - session_start_time
    
    # Znajdź frontiersy dla wyświetlenia
    frontiers = smart_exploration.find_frontiers(terrain_map)
    
    return {
        'coverage_percent': round(coverage_percent, 1),
        'explored_points': explored_points,
        'total_points': total_points,
        'session_time_minutes': round(session_time / 60, 1),
        'distance_traveled_meters': round(total_distance_traveled, 2),
        'obstacles_detected': obstacles_detected,
        'frontiers_count': len(frontiers),
        'exploration_efficiency': round(explored_points / (session_time / 60), 1) if session_time > 0 else 0
    }

def reset_statistics():
    """Resetuje statystyki eksploracji"""
    global session_start_time, total_distance_traveled, obstacles_detected, total_exploration_points
    session_start_time = time.time()
    total_distance_traveled = 0
    obstacles_detected = 0
    total_exploration_points = 0

# Czyszczenie zasobów - ZMODYFIKOWANE
def cleanup():
    global stop_auto, auto_thread
    logger.info("Zwalnianie zasobów")
    stop_auto = True
    if auto_thread and auto_thread.is_alive():
        auto_thread.join(timeout=2)
    
    try:
        cam_tof.stop()
        time.sleep(0.2)
        cam_tof.close()
        stop()
        GPIO.cleanup()
        
        # === NOWE: Zamknij połączenie z InfluxDB ===
        influx_manager.close()
        
    except Exception as e:
        logger.error(f"Błąd podczas zamykania: {e}")
    logger.info("Zasoby zwolnione")

atexit.register(cleanup)
def signal_handler(sig, frame):
    logger.info("Przechwycono Ctrl+C")
    cleanup()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Funkcje ToF (bez zmian)
def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < current_confidence] = (0, 0, 0)
    return preview

def detect_obstacle(depth_data):
    mask = (depth_data > 0) & (depth_data < OBSTACLE_THRESHOLD)
    return np.any(mask)

def generate_depth_frames():
    while True:
        try:
            start_time = time.time()
            frame = cam_tof.requestFrame(2500)
            if frame and isinstance(frame, ac.DepthData):
                depth_buf = frame.depth_data
                confidence_buf = frame.confidence_data
                result_image = (depth_buf * (255.0 / MAX_DISTANCE)).astype(np.uint8)
                result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
                result_image = getPreviewRGB(result_image, confidence_buf)
                result_image = cv2.resize(result_image, (960, 720))
                ret, buffer = cv2.imencode('.jpg', result_image, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                cam_tof.releaseFrame(frame)
                logger.debug(f"Depth frame: {time.time() - start_time:.3f}s")
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"Błąd w generate_depth_frames: {e}")
            break

def generate_confidence_frames():
    while True:
        try:
            start_time = time.time()
            frame = cam_tof.requestFrame(2500)
            if frame and isinstance(frame, ac.DepthData):
                confidence_buf = frame.confidence_data
                confidence_image = cv2.normalize(confidence_buf, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                confidence_image = cv2.cvtColor(confidence_image, cv2.COLOR_GRAY2BGR)
                confidence_image = cv2.resize(confidence_image, (960, 720))
                ret, buffer = cv2.imencode('.jpg', confidence_image, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                cam_tof.releaseFrame(frame)
                logger.debug(f"Confidence frame: {time.time() - start_time:.3f}s")
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"Błąd w generate_confidence_frames: {e}")
            break

# Rozszerzona strona HTML z mapą i statystykami (bez zmian)
HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Terrain Mapping & Robot Control</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.4/css/all.min.css">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f4f9;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 {
            color: #333;
            font-size: 36px;
            margin-bottom: 20px;
        }
        .db-status {
            background-color: #e8f4f8;
            padding: 10px;
            margin: 10px auto;
            border-radius: 5px;
            max-width: 800px;
            font-size: 16px;
            border-left: 4px solid #4CAF50;
        }
        .db-status.error {
            background-color: #ffe6e6;
            border-left-color: #f44336;
        }
        .main-container {
            display: flex;
            flex-wrap: wrap;
            justify-content: center;
            gap: 20px;
        }
        .video-container {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }
        .video-box {
            background-color: #fff;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .video-box h3 {
            margin: 10px 0;
            color: #555;
            font-size: 24px;
        }
        .tof-img {
            width: 480px;
            height: 360px;
            border: 2px solid #ddd;
            border-radius: 4px;
        }
        .map-container {
            background-color: #fff;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
            min-width: 500px;
        }
        .map-container h3 {
            margin: 10px 0;
            color: #555;
            font-size: 24px;
        }
        #terrain-map {
            border: 2px solid #ddd;
            border-radius: 4px;
            background-color: #000;
        }
        .controls {
            margin: 20px auto;
            max-width: 1200px;
        }
        .mode-controls {
            margin: 20px 0;
            padding: 20px;
            background-color: #fff;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        .mode-button {
            padding: 15px 30px;
            margin: 10px;
            font-size: 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            color: white;
        }
        .mode-button.manual {
            background-color: #4CAF50;
        }
        .mode-button.auto {
            background-color: #FF9800;
        }
        .mode-button.active {
            box-shadow: 0 0 15px rgba(0,0,0,0.3);
            transform: scale(1.05);
        }
        .status-display {
            margin: 10px 0;
            padding: 10px;
            background-color: #e8f4f8;
            border-radius: 5px;
            font-size: 18px;
        }
        .statistics-panel {
            background-color: #fff;
            padding: 20px;
            margin: 20px auto;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
            max-width: 1200px;
        }
        .statistics-panel h3 {
            margin: 0 0 15px 0;
            color: #555;
            font-size: 24px;
            text-align: center;
        }
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin: 15px 0;
        }
        .stat-item {
            text-align: center;
            padding: 15px;
            background-color: #f8f9fa;
            border-radius: 8px;
            border-left: 4px solid #4CAF50;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #2c3e50;
            margin-bottom: 5px;
        }
        .stat-label {
            font-size: 14px;
            color: #7f8c8d;
            text-transform: uppercase;
        }
        .controls label {
            display: block;
            margin: 10px 0 5px;
            font-size: 20px;
        }
        .controls input[type="range"] {
            width: 400px;
            height: 10px;
        }
        #confidence-value {
            font-weight: bold;
            font-size: 20px;
        }
        .servo-controls {
            display: grid;
            grid-template-areas: 
                ". up ."
                "left . right"
                ". down .";
            gap: 10px;
            justify-content: center;
            width: 200px;
            margin: 20px auto;
        }
        .servo-controls button {
            width: 100px;
            height: 100px;
            font-size: 32px;
            cursor: pointer;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 50%;
        }
        .servo-controls button:hover {
            background-color: #45a049;
        }
        .up { grid-area: up; }
        .down { grid-area: down; }
        .left { grid-area: left; }
        .right { grid-area: right; }
        .joystick-container {
            margin: 20px auto;
        }
        .joystick {
            width: 300px;
            height: 300px;
            background-color: #ddd;
            border-radius: 50%;
            position: relative;
            margin: 20px auto;
            touch-action: none;
        }
        .joystick-button {
            width: 90px;
            height: 90px;
            background-color: #555;
            border-radius: 50%;
            position: absolute;
            top: 105px;
            left: 105px;
        }
        .control-button {
            padding: 15px 30px;
            margin: 10px;
            font-size: 20px;
            border: none;
            border-radius: 5px;
            background-color: #F44336;
            color: white;
            cursor: pointer;
        }
        .control-button:hover {
            background-color: #d32f2f;
        }
        .disabled {
            opacity: 0.5;
            pointer-events: none;
        }
        .legend {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin: 10px 0;
            font-size: 14px;
        }
        .legend-item {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .legend-color {
            width: 20px;
            height: 20px;
            border-radius: 3px;
        }
    </style>
</head>
<body>
    <h1>Terrain Mapping & Robot Control - v2 + InfluxDB</h1>
    
    <div class="db-status" id="db-status">
        📊 InfluxDB: Connecting...
    </div>
    
    <div class="mode-controls">
        <h3>Control mode</h3>
        <button id="manual-mode" class="mode-button manual active" onclick="setMode('manual')">
            <i class="fas fa-gamepad"></i> Manual mode
        </button>
        <button id="auto-mode" class="mode-button auto" onclick="setMode('auto')">
            <i class="fas fa-robot"></i> Autonomous mode
        </button>
        <div class="status-display" id="status">
            Status: Manual mode - Ready
        </div>
    </div>

    <!-- Panel statystyk -->
    <div class="statistics-panel">
        <h3>📊 Real-time exploration statistics</h3>
        <div class="stats-grid">
            <div class="stat-item">
                <div class="stat-value" id="coverage-percent">0%</div>
                <div class="stat-label">Map coverage</div>
            </div>
            <div class="stat-item">
                <div class="stat-value" id="session-time">0 min</div>
                <div class="stat-label">Session time</div>
            </div>
            <div class="stat-item">
                <div class="stat-value" id="distance-traveled">0.0 m</div>
                <div class="stat-label">Distance traveled</div>
            </div>
            <div class="stat-item">
                <div class="stat-value" id="obstacles-detected">0</div>
                <div class="stat-label">Obstacles detected</div>
            </div>
            <div class="stat-item">
                <div class="stat-value" id="frontiers-count">0</div>
                <div class="stat-label">Exploration Goals</div>
            </div>
            <div class="stat-item">
                <div class="stat-value" id="exploration-efficiency">0</div>
                <div class="stat-label">Efficiency (pkt/min)</div>
            </div>
        </div>
        <button onclick="resetStatistics()" class="control-button" style="background-color: #FF5722; margin-top: 10px;">
            <i class="fas fa-refresh"></i> Statistics reset
        </button>
    </div>

    <div class="main-container">
        <div class="video-container">
            <div class="video-box">
                <h3>Depth map (ToF)</h3>
                <img src="/video_feed" class="tof-img">
            </div>
            <div class="video-box">
                <h3>Confidence Map (ToF)</h3>
                <img src="/video_feed_confidence" class="tof-img">
            </div>
        </div>
        
        <div class="map-container">
            <h3>Terrain Map - Smart Exploration</h3>
            <canvas id="terrain-map" width="500" height="500"></canvas>
            <div class="legend">
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #000;"></div>
                    <span>Unexplored</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #00f;"></div>
                    <span>Close (0-1m)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #0f0;"></div>
                    <span>Average distance (1-2m)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #ff0;"></div>
                    <span>Far (2m+)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #f00;"></div>
                    <span>Robot</span>
                </div>
            </div>
        </div>
    </div>

    <div class="controls">
        <label for="confidence-slider">Confidence value (0-100): <span id="confidence-value">50</span></label>
        <input type="range" id="confidence-slider" min="0" max="100" value="50">
        
        <div id="manual-controls">
            <div class="servo-controls">
                <button class="up" onclick="moveServo(0, 'up')"><i class="fas fa-arrow-up"></i></button>
                <button class="down" onclick="moveServo(0, 'down')"><i class="fas fa-arrow-down"></i></button>
                <button class="left" onclick="moveServo(1, 'left')"><i class="fas fa-arrow-left"></i></button>
                <button class="right" onclick="moveServo(1, 'right')"><i class="fas fa-arrow-right"></i></button>
            </div>
            <div class="joystick-container">
                <div class="joystick" id="joystick">
                    <div class="joystick-button" id="joystick-button"></div>
                </div>
                <button class="control-button" id="stop">STOP</button>
            </div>
        </div>
    </div>
    
    <script>
        const socket = io();
        let currentMode = 'manual';
        let mapCanvas = document.getElementById('terrain-map');
        let mapCtx = mapCanvas.getContext('2d');
        
        // Sprawdź status InfluxDB
        fetch('/api/influxdb_status')
            .then(response => response.json())
            .then(data => {
                const statusEl = document.getElementById('db-status');
                if (data.connected) {
                    statusEl.innerHTML = '✅ InfluxDB: Connected -Data recorded in real time';
                    statusEl.classList.remove('error');
                } else {
                    statusEl.innerHTML = '❌ InfluxDB: Disconnected - ' + (data.error || 'Unknown error');
                    statusEl.classList.add('error');
                }
            })
            .catch(() => {
                document.getElementById('db-status').innerHTML = '❌ InfluxDB: Connection check failed';
                document.getElementById('db-status').classList.add('error');
            });
        
        // Inicjalizacja mapy
        function initMap() {
            mapCtx.fillStyle = '#000';
            mapCtx.fillRect(0, 0, 500, 500);
        }
        
        // Rysowanie mapy
        function drawMap(mapData) {
            mapCtx.clearRect(0, 0, 500, 500);
            mapCtx.fillStyle = '#000';
            mapCtx.fillRect(0, 0, 500, 500);
            
            const terrain = mapData.terrain;
            const robotPos = mapData.robot_pos;
            const path = mapData.path;
            
            // Rysuj teren
            for (let y = 0; y < terrain.length; y++) {
                for (let x = 0; x < terrain[y].length; x++) {
                    const distance = terrain[y][x];
                    if (distance >= 0) {
                        let color;
                        if (distance < 100) color = '#0000ff';      // Blisko - niebieski
                        else if (distance < 200) color = '#00ff00'; // Średnio - zielony
                        else color = '#ffff00';                     // Daleko - żółty
                        
                        mapCtx.fillStyle = color;
                        mapCtx.fillRect(x * 5, y * 5, 5, 5);
                    }
                }
            }
            
            // Rysuj ścieżkę robota
            if (path && path.length > 1) {
                mapCtx.strokeStyle = '#ff8800';
                mapCtx.lineWidth = 2;
                mapCtx.beginPath();
                mapCtx.moveTo(path[0][0] * 5 + 2.5, path[0][1] * 5 + 2.5);
                for (let i = 1; i < path.length; i++) {
                    mapCtx.lineTo(path[i][0] * 5 + 2.5, path[i][1] * 5 + 2.5);
                }
                mapCtx.stroke();
            }
            
            // Rysuj robota
            if (robotPos) {
                const robotX = robotPos.x * 5 + 2.5;
                const robotY = robotPos.y * 5 + 2.5;
                const angle = robotPos.angle * Math.PI / 180;
                
                // Ciało robota
                mapCtx.fillStyle = '#ff0000';
                mapCtx.beginPath();
                mapCtx.arc(robotX, robotY, 8, 0, 2 * Math.PI);
                mapCtx.fill();
                
                // Kierunek robota
                mapCtx.strokeStyle = '#ffffff';
                mapCtx.lineWidth = 3;
                mapCtx.beginPath();
                mapCtx.moveTo(robotX, robotY);
                mapCtx.lineTo(robotX + 15 * Math.sin(angle), robotY - 15 * Math.cos(angle));
                mapCtx.stroke();
            }
        }
        
        // Aktualizacja statystyk
        function updateStatistics(stats) {
            if (!stats) return;
            
            document.getElementById('coverage-percent').textContent = stats.coverage_percent + '%';
            document.getElementById('session-time').textContent = stats.session_time_minutes + ' min';
            document.getElementById('distance-traveled').textContent = stats.distance_traveled_meters + ' m';
            document.getElementById('obstacles-detected').textContent = stats.obstacles_detected;
            document.getElementById('frontiers-count').textContent = stats.frontiers_count;
            document.getElementById('exploration-efficiency').textContent = stats.exploration_efficiency;
        }
        
        // Reset statystyk
        function resetStatistics() {
            if (confirm('Czy na pewno chcesz zresetować statystyki?')) {
                fetch('/reset_statistics', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'}
                })
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'success') {
                        // Wyzeruj wyświetlane wartości
                        document.getElementById('coverage-percent').textContent = '0%';
                        document.getElementById('session-time').textContent = '0 min';
                        document.getElementById('distance-traveled').textContent = '0.0 m';
                        document.getElementById('obstacles-detected').textContent = '0';
                        document.getElementById('frontiers-count').textContent = '0';
                        document.getElementById('exploration-efficiency').textContent = '0';
                    }
                });
            }
        }
        
        // Obsługa WebSocket
        socket.on('map_update', function(data) {
            drawMap(data);
            updateStatistics(data.statistics);
        });
        
        socket.on('status_update', function(data) {
            document.getElementById('status').innerText = 'Status: ' + data.message;
        });
        
        // Przełączanie trybu
        function setMode(mode) {
            fetch('/set_mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mode: mode})
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    currentMode = mode;
                    updateUI();
                }
            });
        }
        
        function updateUI() {
            const manualBtn = document.getElementById('manual-mode');
            const autoBtn = document.getElementById('auto-mode');
            const manualControls = document.getElementById('manual-controls');
            const status = document.getElementById('status');
            
            if (currentMode === 'manual') {
                manualBtn.classList.add('active');
                autoBtn.classList.remove('active');
                manualControls.classList.remove('disabled');
                status.innerText = 'Status: Manual mode - Ready';
            } else {
                manualBtn.classList.remove('active');
                autoBtn.classList.add('active');
                manualControls.classList.add('disabled');
                status.innerText = 'Status: Autonomous mode - Inteligent eksploration';
            }
        }
        
        // Sterowanie servo
        function moveServo(channel, direction) {
            if (currentMode !== 'manual') return;
            fetch('/move_servo', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({channel: channel, direction: direction})
            });
        }
        
        // Regulacja ufności
        document.getElementById('confidence-slider').addEventListener('input', function() {
            var value = this.value;
            document.getElementById('confidence-value').innerText = value;
            fetch('/set_confidence', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({confidence: parseInt(value)})
            });
        });
        
        // Joystick (tylko w trybie manualnym)
        const joystick = document.getElementById('joystick');
        const joystickButton = document.getElementById('joystick-button');
        const stopButton = document.getElementById('stop');
        let active = false;
        let direction = '';
        
        function sendCommand(cmd) {
            if (currentMode !== 'manual') return;
            fetch('/control?cmd=' + cmd)
                .then(response => response.text())
                .then(data => console.log(data));
        }
        
        function handleMove(e) {
            if (!active || currentMode !== 'manual') return;
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            let clientX, clientY;
            
            if (e.type === 'mousemove') {
                clientX = e.clientX - rect.left;
                clientY = e.clientY - rect.top;
            } else if (e.type === 'touchmove') {
                clientX = e.touches[0].clientX - rect.left;
                clientY = e.touches[0].clientY - rect.top;
            }
            
            const x = clientX - centerX;
            const y = clientY - centerY;
            const distance = Math.min(Math.sqrt(x*x + y*y), centerX - 45);
            const angle = Math.atan2(y, x);
            const newX = centerX + distance * Math.cos(angle) - 45;
            const newY = centerY + distance * Math.sin(angle) - 45;
            
            joystickButton.style.left = newX + 'px';
            joystickButton.style.top = newY + 'px';
            
            let newDirection = '';
            if (distance > 60) {
                if (Math.abs(x) > Math.abs(y)) {
                    newDirection = x > 0 ? 'right' : 'left';
                } else {
                    newDirection = y > 0 ? 'backward' : 'forward';
                }
            } else {
                newDirection = 'stop';
            }
            
            if (newDirection !== direction) {
                direction = newDirection;
                sendCommand(direction);
            }
        }
        
        function handleStart(e) {
            if (currentMode !== 'manual') return;
            active = true;
            if (e.type === 'touchstart') {
                e.preventDefault();
            }
            handleMove(e);
        }
        
        function handleEnd() {
            active = false;
            direction = 'stop';
            if (currentMode === 'manual') {
                sendCommand('stop');
            }
            joystickButton.style.left = '105px';
            joystickButton.style.top = '105px';
        }
        
        // Event listenery dla joysticka
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);
        joystick.addEventListener('touchstart', handleStart);
        joystick.addEventListener('touchmove', handleMove);
        joystick.addEventListener('touchend', handleEnd);
        
        stopButton.addEventListener('click', () => sendCommand('stop'));
        stopButton.addEventListener('touchstart', (e) => {
            e.preventDefault();
            sendCommand('stop');
        });
        
        // Inicjalizacja
        initMap();
        updateUI();
    </script>
</body>
</html>
"""

# Trasy Flask - ZMODYFIKOWANE i NOWE
@app.route('/')
def index():
    return HTML

@app.route('/video_feed')
def video_feed():
    return Response(generate_depth_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_confidence')
def video_feed_confidence():
    return Response(generate_confidence_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_confidence', methods=['POST'])
def set_confidence():
    try:
        global current_confidence
        data = request.get_json()
        current_confidence = data['confidence']
        logger.info(f"Ufność: {current_confidence}")
        return jsonify({"status": "success"})
    except Exception as e:
        logger.error(f"Błąd w set_confidence: {e}")
        return jsonify({"status": "error"})

@app.route('/set_mode', methods=['POST'])
def set_mode():
    try:
        global auto_mode, auto_thread, stop_auto
        data = request.get_json()
        new_mode = data['mode']
        
        if new_mode == 'auto' and not auto_mode:
            # Włącz tryb automatyczny
            auto_mode = True
            stop_auto = False
            auto_thread = threading.Thread(target=auto_explore)
            auto_thread.daemon = True
            auto_thread.start()
            logger.info("Włączono tryb automatyczny z inteligentną eksploracją")
            socketio.emit('status_update', {'message': 'Tryb automatyczny - Inteligentna eksploracja'})
            
        elif new_mode == 'manual' and auto_mode:
            # Wyłącz tryb automatyczny
            auto_mode = False
            stop_auto = True
            if auto_thread and auto_thread.is_alive():
                auto_thread.join(timeout=2)
            stop()
            logger.info("Włączono tryb manualny")
            socketio.emit('status_update', {'message': 'Tryb manualny - Gotowy'})
        
        return jsonify({"status": "success", "mode": new_mode})
    except Exception as e:
        logger.error(f"Błąd w set_mode: {e}")
        return jsonify({"status": "error"})

@app.route('/move_servo', methods=['POST'])
def move_servo():
    try:
        if auto_mode:
            return jsonify({"status": "auto mode active"})
            
        data = request.get_json()
        channel = data['channel']
        direction = data['direction']
        current_angle = pwm.servo_angles[channel]
        step = 10
        
        if channel == 0:
            new_angle = max(10, current_angle - step) if direction == 'up' else min(170, current_angle + step)
        elif channel == 1:
            new_angle = min(170, current_angle + step) if direction == 'left' else max(10, current_angle - step)
        else:
            return jsonify({"status": "invalid channel"})
        
        pwm.setServoAngle(channel, new_angle)
        return jsonify({"status": "success", "angle": new_angle})
    except Exception as e:
        logger.error(f"Błąd w move_servo: {e}")
        return jsonify({"status": "error"})

@app.route('/control')
def control():
    try:
        if auto_mode:
            return "Auto mode active"
            
        cmd = request.args.get('cmd', 'stop')
        if cmd == 'forward':
            forward()
            update_robot_position('forward')  # ZMODYFIKOWANE: Dodano update pozycji
        elif cmd == 'backward':
            backward()
            update_robot_position('backward')
        elif cmd == 'left':
            left()
            update_robot_position('left')
        elif cmd == 'right':
            right()
            update_robot_position('right')
        else:
            stop()
        return f"Command: {cmd}"
    except Exception as e:
        logger.error(f"Błąd w control: {e}")
        return f"Error: {str(e)}"

@app.route('/get_map')
def get_map():
    """Zwraca aktualną mapę terenu"""
    try:
        map_data = {
            'terrain': terrain_map.tolist(),
            'robot_pos': {'x': robot_x, 'y': robot_y, 'angle': robot_angle},
            'path': list(robot_path)[-100:],
            'timestamp': time.time(),
            'statistics': get_exploration_statistics()
        }
        return jsonify(map_data)
    except Exception as e:
        logger.error(f"Błąd w get_map: {e}")
        return jsonify({"status": "error"})

@app.route('/get_statistics')
def get_statistics():
    """Zwraca statystyki eksploracji"""
    try:
        stats = get_exploration_statistics()
        return jsonify(stats)
    except Exception as e:
        logger.error(f"Błąd w get_statistics: {e}")
        return jsonify({"status": "error"})

@app.route('/reset_statistics', methods=['POST'])
def reset_statistics_endpoint():
    """Resetuje statystyki eksploracji"""
    try:
        reset_statistics()
        logger.info("Zresetowano statystyki")
        return jsonify({"status": "success"})
    except Exception as e:
        logger.error(f"Błąd w reset_statistics: {e}")
        return jsonify({"status": "error"})

@app.route('/clear_map', methods=['POST'])
def clear_map():
    """Czyści mapę terenu"""
    try:
        global terrain_map, robot_path
        terrain_map.fill(-1)
        robot_path.clear()
        reset_statistics()
        send_map_update()
        logger.info("Wyczyszczono mapę i zresetowano statystyki")
        return jsonify({"status": "success"})
    except Exception as e:
        logger.error(f"Błąd w clear_map: {e}")
        return jsonify({"status": "error"})

# === NOWE: API endpoints dla InfluxDB ===
@app.route('/api/influxdb_status')
def influxdb_status():
    """Sprawdź status połączenia z InfluxDB"""
    return jsonify({
        "connected": influx_manager.connected,
        "config": {
            "url": INFLUXDB_CONFIG['url'],
            "org": INFLUXDB_CONFIG['org'],
            "bucket": INFLUXDB_CONFIG['bucket'],
            "robot_id": INFLUXDB_CONFIG['robot_id']
        }
    })

@app.route('/api/robot_data/<robot_id>')
def get_robot_data_from_db(robot_id):
    """API endpoint do pobierania danych z InfluxDB dla innych komputerów"""
    try:
        # Pobierz najnowszy stan robota
        robot_state = influx_manager.get_latest_robot_state(robot_id)
        
        # Pobierz punkty terenu z ostatniej godziny
        terrain_points = influx_manager.get_terrain_map_live(robot_id, last_minutes=60)
        
        # Konwertuj dane do formatu compatible z WebSocket
        map_data = {
            "robot_state": robot_state,
            "terrain_points": terrain_points,
            "timestamp": time.time(),
            "source": "influxdb"
        }
        
        return jsonify(map_data)
        
    except Exception as e:
        logger.error(f"Błąd pobierania danych z InfluxDB: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/force_save_to_db', methods=['POST'])
def force_save_to_db():
    """Wymusza zapis danych do InfluxDB (dla testów)"""
    try:
        save_robot_state_to_db()
        save_terrain_data_to_db()
        save_exploration_stats_to_db()
        
        return jsonify({
            "status": "success",
            "message": "Dane zapisane do InfluxDB",
            "timestamp": time.time()
        })
        
    except Exception as e:
        logger.error(f"Błąd wymuszenia zapisu do DB: {e}")
        return jsonify({"status": "error", "error": str(e)}), 500

# WebSocket events
@socketio.on('connect')
def handle_connect():
    logger.info('Klient połączony')
    # Wyślij aktualną mapę do nowego klienta
    send_map_update()

@socketio.on('disconnect')
def handle_disconnect():
    logger.info('Klient rozłączony')

@socketio.on('request_map')
def handle_request_map():
    send_map_update()

if __name__ == '__main__':
    try:
        stop()  # Zatrzymaj silniki na starcie
        logger.info("Uruchamianie serwera z WebSocket - Wersja 2 z inteligentną eksploracją + InfluxDB")
        
        # Test połączenia z InfluxDB na starcie
        if influx_manager.connected:
            logger.info("✅ Serwer gotowy z integracją InfluxDB")
        else:
            logger.warning("⚠️ Serwer działa bez InfluxDB - dane nie będą zapisywane")
            
        socketio.run(app, host="0.0.0.0", port=80, debug=False)
    except Exception as e:
        logger.error(f"Błąd aplikacji: {e}")
        cleanup()
