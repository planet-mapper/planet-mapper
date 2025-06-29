
# 🤖 Autonomous Terrain Mapping Robot

Intelligent robot for real-time terrain mapping - designed to explore unknown areas such as alien planets 🪐 or disaster zones on Earth 🌍.

![robot_mapper](https://github.com/user-attachments/assets/1a2f06a2-8423-495e-a092-db9826d62f56)

## 🎯 Project Goal

The robot autonomously explores and maps unknown terrain, creating detailed 3D maps in real-time. Ideal for exploratory missions where human presence is impossible or dangerous.

## 🌟 Key Features

### 🧠 Intelligent Exploration

- **Frontier-based Algorithm** - robot automatically finds and explores uncharted areas
- **Purposeful Movement** - instead of random driving, robot has specific exploration goals
- **Adaptive Planning** - dynamic strategy adjustment to encountered terrain

### 📡 Real-time Mapping

- **ToF Camera (Time-of-Flight)** - precise distance measurements up to 4 meters
- **100×100 point grid** with 10cm × 10cm resolution
- **Live Visualization** - map updated in real-time
- **Color Coding**: blue (close), green (medium), yellow (far)

### 🌐 Web Interface

- **Remote Control** via WiFi from any device
- **Mode Switching**: manual ↔ automatic
- **Statistics Panel**: terrain coverage, exploration time, distance traveled
- **Interactive Map** with robot movement history

### 🚀 Advanced Algorithms

- **Obstacle Detection** - automatic barrier avoidance
- **Route Optimization** - intelligent exploration direction selection
- **Frontier Clustering** - grouping small unexplored areas

## 🛠️ Technical Specifications

### Hardware

- **Platform**: Raspberry Pi 5 (ARM)
- **Main Sensor**: Arducam ToF (Time-of-Flight) Camera
- **Drive**: 4× SJ01 geared motors 120:1 6V 160RPM with encoders + red LM298-based controllers (with heat sinks)
- **Servo**: 2× servomechanisms (PCA9685)
- **Connectivity**: WiFi 802.11n
- **Range**: ToF up to 4m, WiFi up to 100m

### Software Stack
```
┌─────────────────────────────────────┐
│           Web Interface             │ (HTML5 + JavaScript + WebSocket)
├─────────────────────────────────────┤
│          Flask + SocketIO           │ (Python Web Framework)
├─────────────────────────────────────┤
│      Mapping & Navigation AI        │ (NumPy + OpenCV)
├─────────────────────────────────────┤
│     Hardware Control Layer         │ (RPi.GPIO + Arducam SDK)
└─────────────────────────────────────┘
```

## 📊 AI Algorithms

### 1. Frontier-Based Exploration
```python
def find_frontiers(terrain_map):
    """Find boundaries between explored and unexplored areas"""
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if terrain_map[y,x] == -1:  # Unexplored
                if has_explored_neighbors(x, y):
                    frontiers.append((x, y))
```

### 2. Intelligent Target Selection
```python
def choose_next_target(robot_pos, frontiers):
    """Choose optimal exploration target"""
    return min(frontiers, key=lambda f: distance(robot_pos, f))
```

### 3. Obstacle Detection & Avoidance
```python
def detect_critical_obstacle(depth_data):
    """Detect obstacles < 10cm and execute escape maneuver"""
    if min_distance < CRITICAL_DISTANCE:
        execute_escape_sequence()
```

## 🚀 Quick Start

### 1. Installation
```bash
# Clone repository
git clone https://github.com/[your-username]/autonomous-terrain-robot.git
cd autonomous-terrain-robot

# Install dependencies
pip install -r requirements.txt

# Hardware configuration
sudo raspi-config  # Enable I2C, Camera, GPIO
```

### 2. Launch
```bash
# Start robot
sudo python3 autonomous_v2.py

# Open interface in browser
http://[ROBOT_IP]:80
```

### 3. Control

- **Manual Mode**: Joystick + servo control
- **Automatic Mode**: Robot explores terrain autonomously
- **Monitoring**: Observe statistics and mapping progress

## 📈 Real-time Statistics

| Metric | Description | Example |
|--------|-------------|---------|
| Terrain Coverage | % of explored area | 67.3% |
| Session Time | Active exploration time | 23.5 min |
| Distance | Total distance traveled | 45.7 m |
| Obstacles | Detected barriers | 12 |
| Targets | Remaining areas | 8 |
| Efficiency | Points/minute | 156 |

## 🌍 Use Cases

### 🪐 Planetary Exploration

- **Mars/Moon**: Surface mapping before sending astronauts
- **Asteroids**: Composition and topography analysis of small celestial bodies
- **Satellites**: Studying areas inaccessible to orbiters

### 🌋 Earth Rescue Missions

- **Disaster Zones**: Earthquakes, floods, fires
- **Contaminated Areas**: Nuclear accidents, chemical spills
- **Hostile Regions**: Caves, glaciers, volcanoes

### 🔬 Scientific Research

- **Archaeology**: Site mapping without damage
- **Geology**: Rock formation analysis
- **Ecology**: Natural environment monitoring

## 🧪 Demo & Examples

### Exploration Visualization
🤖 Start → 🔍 Scan → 🎯 Target → 🚀 Move → 📊 Map → 🔄 Repeat

### Sample Session

- **00:00** - Robot starts at point (50,50)
- **00:15** - 8 frontiers detected for exploration
- **02:30** - First obstacle - avoidance maneuver
- **05:45** - 25% of terrain mapped
- **12:00** - All main areas explored
- **15:30** - Patrol mode - exploration complete

## 🔧 Configuration

### Exploration Parameters
```python
MAP_SIZE = 100           # Map size (100×100)
OBSTACLE_THRESHOLD = 700 # Obstacle detection threshold (mm)
CRITICAL_DISTANCE = 100  # Emergency distance (mm)
CONFIDENCE_VALUE = 50    # ToF confidence threshold (0-100)
```

### Performance Optimization
```python
JPEG_QUALITY = 80        # Video stream quality
FRAME_RATE = 10          # ToF FPS (Hz)
UPDATE_INTERVAL = 0.5    # Map update frequency (s)
```

## 📋 Requirements

### Hardware

- Raspberry Pi 5 (4GB RAM minimum)
- microSD card (32GB+, Class 10)
- Arducam ToF Camera
- 4× SJ01 geared motors 120:1 6V 160RPM with encoders + red LM298-based motor drivers (with heat sinks)
- PCA9685 servo controller
- 12V/5A power supply

### Software

- Raspbian OS Lite (64-bit)
- Python 3.9+
- OpenCV 4.5+
- Flask + SocketIO
- NumPy, RPi.GPIO

### Network

- WiFi router (for remote control)
- Static IP (recommended)

## 🔄 Project Development

### Version 1.0 ✅

- ✅ Basic exploration
- ✅ ToF mapping
- ✅ Web interface
- ✅ Manual control

### Version 2.0 ✅ (Current)

- ✅ Intelligent exploration (frontier-based)
- ✅ Real-time statistics
- ✅ Algorithm optimization
- ✅ Improved obstacle avoidance

### Version 3.0 🚧 (Planned)

- 🚧 Loop detection + auto-escape
- 🚧 Map export (PNG/JSON/KML)
- 🚧 PostgreSQL database
- 🚧 REST API + documentation

### Version 4.0 💭 (Future)

- 💭 Object recognition (AI)
- 💭 Semantic mapping
- 💭 Multi-robot coordination
- 💭 ROS2 integration

## 🤝 Collaboration

The project is open to collaboration! Welcome contributions:

- 🐛 **Bug Reports** - Issues on GitHub
- 💡 **New Ideas** - Feature requests
- 🔧 **Pull Requests** - Code improvements
- 📚 **Documentation** - Tutorials and examples

### How to Contribute

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📄 License

Project released under MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- **Arducam** - for ToF camera SDK
- **Raspberry Pi Foundation** - for hardware platform
- **OpenCV Community** - for computer vision libraries
- **Flask Team** - for web framework

## 📞 Contact

- **GitHub**: @your-username
- **Email**: your.email@example.com
- **LinkedIn**: Your profile

---

⭐ **If you like the project, leave a star on GitHub!** ⭐

**Robot ready to explore new worlds** 🚀🤖🌌
