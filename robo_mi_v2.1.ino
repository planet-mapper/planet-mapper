#include <WiFi.h>
#include <WebServer.h>
#include <Motoron.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Preferences.h>

// ===== Pinout / UART / I2C =====
#define RX_PIN D7
#define TX_PIN D6
#define SDA_PIN D4
#define SCL_PIN D5
#define BUZZER_PIN D2
#define LED_PIN D1
#define BAUD 115200

MotoronSerial mc;
HardwareSerial& mcSerial = Serial1;

WebServer server(80);
Preferences preferences;

Adafruit_VL53L0X lox;
bool tofAvailable = false;

int distance = -1;

// ===== WiFi AP =====
const char* ssid = "ESP32-Robot";
const char* password = "zaq12wsx";

// ===== Prędkości (regulowane) =====
int speedMax = 800;
int speedForwardBackward = 400;
int speedTurn = 400;

// ===== Stany =====
bool ledState = false;
bool buzzerState = false;

// ===== Autonomiczny tryb =====
bool autonomousMode = false;
int obstacleThreshold = 300;   // mm – próg wykrywania przeszkody (regulowany suwakiem)
int turnTime = 500;            // ms – czas skrętu
unsigned long lastAutonomousAction = 0;
int autonomousSpeed = 300;     // prędkość w trybie auto (regulowana suwakiem)
int minSafeDistance = 200;     // mm – twarda granica bezpieczeństwa
String autonomousStatus = "Stopped";

unsigned long buzzerStartTime = 0;
const int buzzerDuration = 1000;

// ===== Pomocnicze: swap + mediana z 3 =====
void simpleSwap(int &a, int &b) { int t=a; a=b; b=t; }

int medianOf3(int a, int b, int c) {
  if (a > b) simpleSwap(a, b);
  if (b > c) simpleSwap(b, c);
  if (a > b) simpleSwap(a, b);
  return b;
}

// ===== Kierunki jazdy w jednym miejscu =====
void driveStop() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
}

void driveForward(int v) {
  mc.setSpeed(1, v);
  mc.setSpeed(2, v);
}

void driveBackward(int v) {
  mc.setSpeed(1, -v);
  mc.setSpeed(2, -v);
}

// Uwaga: po Twojej kalibracji „prawo/lewo” były zamienione.
// Poniżej mapowanie poprawione zgodnie z Twoją prośbą:
//  - turnLeft(): lewy silnik szybciej do przodu, prawy do tyłu → faktyczny skręt w LEWO
//  - turnRight(): lewy do tyłu, prawy do przodu → faktyczny skręt w PRAWO
void turnLeft(int v) {
  mc.setSpeed(1, v);
  mc.setSpeed(2, -v);
}

void turnRight(int v) {
  mc.setSpeed(1, -v);
  mc.setSpeed(2, v);
}

// ===== Preferences =====
void savePreferences() {
  preferences.begin("robot", false);
  preferences.putInt("speedFB", speedForwardBackward);
  preferences.putInt("speedTurn", speedTurn);
  preferences.putInt("obstThresh", obstacleThreshold);
  preferences.putInt("autoSpeed", autonomousSpeed);
  preferences.putBool("autoMode", autonomousMode);  
  preferences.end();
}

void loadPreferences() {
  preferences.begin("robot", true);
  speedForwardBackward = preferences.getInt("speedFB", 400);
  speedTurn = preferences.getInt("speedTurn", 400);
  obstacleThreshold = preferences.getInt("obstThresh", 300);
  autonomousSpeed = preferences.getInt("autoSpeed", 300);
  autonomousMode = preferences.getBool("autoMode", false);
  preferences.end();
}

// ===== Autonomia =====
void autonomousControl() {
  if (!tofAvailable || !autonomousMode) return;

  VL53L0X_RangingMeasurementData_t measure;
  int d1, d2, d3;

  // Trzy szybkie pomiary i mediana (filtracja szumów / błędów chwilowych)
  lox.rangingTest(&measure, false);
  d1 = (measure.RangeStatus == 0) ? measure.RangeMilliMeter : 8191;
  delay(10);
  lox.rangingTest(&measure, false);
  d2 = (measure.RangeStatus == 0) ? measure.RangeMilliMeter : 8191;
  delay(10);
  lox.rangingTest(&measure, false);
  d3 = (measure.RangeStatus == 0) ? measure.RangeMilliMeter : 8191;

  int filteredDistance = medianOf3(d1, d2, d3);

  if (filteredDistance < 8000) {
    distance = filteredDistance;
    lastAutonomousAction = millis();

    if (distance < minSafeDistance) {
      // Bardzo blisko – STOP + delikatne cofnięcie + skręt
      driveStop();
      autonomousStatus = "Emergency Stop";
      delay(100);

      // Cofanie (naprawdę do tyłu – oba koła wstecz)
      driveBackward(autonomousSpeed);
      delay(300);

      // Skręt (naprzemiennie lewo/prawo by nie „zamurowało”)
      static bool leftFirst = false;
      leftFirst = !leftFirst;
      if (leftFirst) turnLeft(autonomousSpeed);
      else           turnRight(autonomousSpeed);
      delay(turnTime);

      autonomousStatus = "Avoiding - Critical";
    }
    else if (distance < obstacleThreshold) {
      // Wykryto przeszkodę – skręć (naprzemiennie)
      autonomousStatus = "Obstacle detected - Turning";
      static bool turnDirection = false;
      turnDirection = !turnDirection;

      if (turnDirection) {
        // LEWO
        turnLeft(autonomousSpeed);
      } else {
        // PRAWO
        turnRight(autonomousSpeed);
      }
      delay(turnTime);
    }
    else {
      // Droga wolna – jedź prosto
      driveForward(autonomousSpeed);
      autonomousStatus = "Moving Forward";
    }
  }
  else {
    // 8190/8191 – poza zasięgiem; traktuj jako „wolna droga”
    driveForward(autonomousSpeed);
    autonomousStatus = "Moving Forward - Max Range";
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  loadPreferences();

  mcSerial.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  mcSerial.setTimeout(20);
  mc.setPort(&mcSerial);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();

  Wire.begin(SDA_PIN, SCL_PIN);
  if (lox.begin()) {
    tofAvailable = true;
    Serial.println("VL53L0X OK");
  } else {
    Serial.println("Brak ToF");
  }

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);

  // ===== WWW =====
  server.on("/", HTTP_GET, []() {
    String html = R"rawliteral(
      <!DOCTYPE html><html><head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { background-color: black; color: white; font-family: sans-serif; text-align: center; }
        button { font-size: 24px; margin: 10px; width: 100px; height: 100px; }
        input[type=range] { width: 300px; }
        iframe { width: 100%; height: 200px; border: none; margin-bottom: 10px; }
        .autonomous-panel {
          background-color: #222;
          border: 2px solid #444;
          border-radius: 10px;
          padding: 15px;
          margin: 20px auto;
          max-width: 520px;
        }
        .auto-button {
          background-color: #4CAF50;
          color: white;
          padding: 15px 30px;
          font-size: 18px;
          border: none;
          border-radius: 5px;
          cursor: pointer;
          margin: 10px;
        }
        .auto-button.active { background-color: #ff5722; }
        .status { font-size: 16px; color: #0ff; margin: 10px; }
      </style>
      </head><body>
      <h1>Robot WiFi - Autonomous Mode</h1>
      <iframe src="http://192.168.4.2"></iframe>

      <div class="autonomous-panel">
        <h2>🤖 Autonomous Mode</h2>
        <button id="autoBtn" class="auto-button" onclick="toggleAutonomous()">Enable Auto</button>
        <div class="status">Status: <span id="autoStatus">Manual Control</span></div>
        <div class="status">ToF Distance: <span id="tof">--</span> mm</div>
        <h3>Settings:</h3>
        <label for="obstThresh">Obstacle threshold (mm):</label>
        <input type="range" id="obstThresh" min="100" max="1000" value="300" oninput="updateAutoParam('obstThresh', this.value)">
        <span id="obstThreshVal">300</span><br>
        <label for="autoSpeed">Auto Speed:</label>
        <input type="range" id="autoSpeed" min="100" max="600" value="300" oninput="updateAutoParam('autoSpeed', this.value)">
        <span id="autoSpeedVal">300</span>
      </div>

      <div id="manualControls">
        <h2>Manual Control</h2>
        <div>
          <button ontouchstart="send('F')" ontouchend="send('S')" onmousedown="send('F')" onmouseup="send('S')">⬆️</button><br>
          <button ontouchstart="send('L')" ontouchend="send('S')" onmousedown="send('L')" onmouseup="send('S')">⬅️</button>
          <button ontouchstart="send('R')" ontouchend="send('S')" onmousedown="send('R')" onmouseup="send('S')">➡️</button><br>
          <button ontouchstart="send('B')" ontouchend="send('S')" onmousedown="send('B')" onmouseup="send('S')">⬇️</button>
        </div>
        <div>
          <label for="speedFB">Forward/Backward Speed:</label><br>
          <input type="range" id="speedFB" min="0" max="800" value="400" oninput="updateSpeed('speedFB', this.value)"><br>
          <label for="speedTurn">Turn Speed:</label><br>
          <input type="range" id="speedTurn" min="0" max="800" value="400" oninput="updateSpeed('speedTurn', this.value)"><br>
        </div>
      </div>

      <script>
        let isAutonomous = false;

        function send(cmd) { if (!isAutonomous) fetch("/cmd?d=" + cmd); }
        function updateSpeed(name, val) { fetch("/setSpeed?" + name + "=" + val); }

        function toggleAutonomous() {
          isAutonomous = !isAutonomous;
          fetch("/autonomous?enable=" + (isAutonomous ? "1" : "0"))
            .then(() => {
              const btn = document.getElementById("autoBtn");
              if (isAutonomous) { btn.textContent = "Disable Auto"; btn.classList.add("active"); }
              else { btn.textContent = "Enable Auto"; btn.classList.remove("active"); }
            });
        }

        function updateAutoParam(name, val) {
          document.getElementById(name + "Val").textContent = val;
          fetch("/setAutoParam?" + name + "=" + val);
        }

        setInterval(() => {
          fetch("/status").then(r => r.json()).then(data => {
            document.getElementById("tof").innerText = data.distance;
            document.getElementById("autoStatus").innerText = data.autoStatus;
          });
        }, 200);
      </script>
      </body></html>
    )rawliteral";
    server.send(200, "text/html", html);
  });

  // ===== Manualne komendy (z poprawionym lewo/prawo) =====
  server.on("/cmd", HTTP_GET, []() {
    if (!autonomousMode) {
      String d = server.arg("d");
      if      (d == "F") { driveForward(speedForwardBackward); }
      else if (d == "B") { driveBackward(speedForwardBackward); }
      else if (d == "L") { turnLeft(speedTurn); }   // <-- poprawione LEWO
      else if (d == "R") { turnRight(speedTurn); }  // <-- poprawione PRAWO
      else               { driveStop(); }
    }
    server.send(200, "text/plain", "OK");
  });

  // ===== Włącz/wyłącz autonomię =====
  server.on("/autonomous", HTTP_GET, []() {
    String enable = server.arg("enable");
    autonomousMode = (enable == "1");
    if (!autonomousMode) {
      driveStop();
      autonomousStatus = "Manual Control";
    } else {
      autonomousStatus = "Autonomous Active";
      digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
    }
    savePreferences();  
    server.send(200, "text/plain", autonomousMode ? "Auto ON" : "Auto OFF");
  });

  // ===== Ustawienia progu/prędkości w autonomii =====
  server.on("/setAutoParam", HTTP_GET, []() {
    if (server.hasArg("obstThresh")) obstacleThreshold = server.arg("obstThresh").toInt();
    if (server.hasArg("autoSpeed"))  autonomousSpeed   = server.arg("autoSpeed").toInt();
    savePreferences();
    server.send(200, "text/plain", "OK");
  });

  // ===== Status JSON =====
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"distance\":\"" + (distance >= 0 ? String(distance) : "--") + "\",";
    json += "\"autonomous\":" + String(autonomousMode ? "true" : "false") + ",";
    json += "\"autoStatus\":\"" + autonomousStatus + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  // ===== Ustawienia prędkości manualnych =====
  server.on("/setSpeed", HTTP_GET, []() {
    if (server.hasArg("speedFB"))   speedForwardBackward = server.arg("speedFB").toInt();
    if (server.hasArg("speedTurn")) speedTurn            = server.arg("speedTurn").toInt();
    savePreferences();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("HTTP server ready");
}

// ===== LOOP =====
void loop() {
  server.handleClient();

  if (autonomousMode && tofAvailable) {
    static unsigned long lastAutoCheck = 0;
    if (millis() - lastAutoCheck > 100) {   // ~10 Hz pętla autonomii
      autonomousControl();
      lastAutoCheck = millis();
    }
  }

  // Safety Stop jeśli długo brak danych z ToF w trybie auto
  if (autonomousMode && tofAvailable && (millis() - lastAutonomousAction > 5000)) {
    driveStop();
    autonomousStatus = "Safety Stop - No Data";
  }
}
