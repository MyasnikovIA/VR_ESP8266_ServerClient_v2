#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

Adafruit_MPU6050 mpu;

// WiFi credentials
const char* ssid = "ESP8266_Network_Monitor";
const char* password = "12345678";

// Новое имя для ESP8266
String newHostname = "VR_Head_Hom_001";


ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Sensor data
float pitch = 0, roll = 0, yaw = 0;
float lastSentPitch = 0, lastSentRoll = 0, lastSentYaw = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
bool calibrated = false;
unsigned long lastTime = 0;

// Относительный ноль
float zeroPitch = 0, zeroRoll = 0, zeroYaw = 0;
bool zeroSet = false;

// Накопленные углы (без ограничений)
double accumulatedPitch = 0, accumulatedRoll = 0, accumulatedYaw = 0;
float prevPitch = 0, prevRoll = 0, prevYaw = 0;
bool firstMeasurement = true;

// WebSocket connection management
bool clientConnected = false;
unsigned long lastDataSend = 0;
const unsigned long SEND_INTERVAL = 50;
const float CHANGE_THRESHOLD = 1.0;

// Установка относительного нуля
void setZeroPoint() {
  zeroPitch = pitch;
  zeroRoll = roll;
  zeroYaw = yaw;
  zeroSet = true;
  
  // Сбрасываем накопленные углы при установке нуля
  accumulatedPitch = 0;
  accumulatedRoll = 0;
  accumulatedYaw = 0;
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
  
  Serial.println("Zero point set");
  Serial.print("Zero Pitch: "); Serial.print(zeroPitch);
  Serial.print(" Roll: "); Serial.print(zeroRoll);
  Serial.print(" Yaw: "); Serial.println(zeroYaw);
  
  String message = "ZERO_SET:PITCH:" + String(zeroPitch, 2) + 
                   ",ROLL:" + String(zeroRoll, 2) + 
                   ",YAW:" + String(zeroYaw, 2);
  webSocket.broadcastTXT(message);
}

// Сброс относительного нуля
void resetZeroPoint() {
  zeroPitch = 0;
  zeroRoll = 0;
  zeroYaw = 0;
  zeroSet = false;
  
  accumulatedPitch = 0;
  accumulatedRoll = 0;
  accumulatedYaw = 0;
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
  
  Serial.println("Zero point reset");
  webSocket.broadcastTXT("ZERO_RESET");
}

// Расчет накопленных углов (без ограничений)
void updateAccumulatedAngles() {
  if (firstMeasurement) {
    prevPitch = pitch;
    prevRoll = roll;
    prevYaw = yaw;
    firstMeasurement = false;
    return;
  }
  
  // Вычисляем разницу углов с учетом переходов через 180/-180
  float deltaPitch = pitch - prevPitch;
  float deltaRoll = roll - prevRoll;
  float deltaYaw = yaw - prevYaw;
  
  // Корректируем разницу для переходов через границу ±180
  if (deltaPitch > 180) deltaPitch -= 360;
  else if (deltaPitch < -180) deltaPitch += 360;
  
  if (deltaRoll > 180) deltaRoll -= 360;
  else if (deltaRoll < -180) deltaRoll += 360;
  
  if (deltaYaw > 180) deltaYaw -= 360;
  else if (deltaYaw < -180) deltaYaw += 360;
  
  // Накопление углов
  accumulatedPitch += deltaPitch;
  accumulatedRoll += deltaRoll;
  accumulatedYaw += deltaYaw;
  
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
}

// Получение относительных углов (без ограничений)
double getRelativePitch() {
  if (!zeroSet) return accumulatedPitch;
  return accumulatedPitch - zeroPitch;
}

double getRelativeRoll() {
  if (!zeroSet) return accumulatedRoll;
  return accumulatedRoll - zeroRoll;
}

double getRelativeYaw() {
  if (!zeroSet) return accumulatedYaw;
  return accumulatedYaw - zeroYaw;
}

void calibrateSensor() {
  Serial.println("Calibrating...");
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(2);
  }
  
  gyroOffsetX = sumX / 500;
  gyroOffsetY = sumY / 500;
  gyroOffsetZ = sumZ / 500;
  calibrated = true;
  
  Serial.println("Calibration complete");
}

void sendSensorData() {
  // Обновляем накопленные углы
  updateAccumulatedAngles();
  
  // Получаем относительные углы
  double relPitch = getRelativePitch();
  double relRoll = getRelativeRoll();
  double relYaw = getRelativeYaw();
  
  String data = "PITCH:" + String(pitch, 1) + 
                ",ROLL:" + String(roll, 1) + 
                ",YAW:" + String(yaw, 1) +
                ",REL_PITCH:" + String(relPitch, 2) +
                ",REL_ROLL:" + String(relRoll, 2) +
                ",REL_YAW:" + String(relYaw, 2) +
                ",ACC_PITCH:" + String(accumulatedPitch, 2) +
                ",ACC_ROLL:" + String(accumulatedRoll, 2) +
                ",ACC_YAW:" + String(accumulatedYaw, 2) +
                ",ZERO_SET:" + String(zeroSet ? "true" : "false");
  
  webSocket.broadcastTXT(data);
  lastSentPitch = pitch;
  lastSentRoll = roll;
  lastSentYaw = yaw;
}

bool dataChanged() {
  return (abs(pitch - lastSentPitch) >= CHANGE_THRESHOLD ||
          abs(roll - lastSentRoll) >= CHANGE_THRESHOLD ||
          abs(yaw - lastSentYaw) >= CHANGE_THRESHOLD);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      clientConnected = (webSocket.connectedClients() > 0);
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        clientConnected = true;
        sendSensorData();
      }
      break;
      
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        Serial.printf("[%u] Received: %s\n", num, message);
        
        if (message == "GET_DATA") {
          sendSensorData();
        }
        else if (message == "RECALIBRATE") {
          calibrated = false;
          calibrateSensor();
          String calMessage = "RECALIBRATION_COMPLETE";
          webSocket.broadcastTXT(calMessage);
        }
        else if (message == "RESET_ANGLES") {
          pitch = 0; roll = 0; yaw = 0;
          lastSentPitch = 0; lastSentRoll = 0; lastSentYaw = 0;
          resetZeroPoint();
          String resetMessage = "ANGLES_RESET";
          webSocket.broadcastTXT(resetMessage);
          sendSensorData();
        }
        else if (message == "SET_ZERO") {
          setZeroPoint();
          webSocket.broadcastTXT("ZERO_POINT_SET");
        }
        else if (message == "RESET_ZERO") {
          resetZeroPoint();
          webSocket.broadcastTXT("ZERO_POINT_RESET");
        }
      }
      break;
  }
}

void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.sendHeader("Content-Type", "text/html; charset=utf-8");
}

void handleOptions() {
  addCORSHeaders();
  server.send(200, "text/plain", "");
}

void handleAPIStatus() {
  addCORSHeaders();
  String json = "{";
  json += "\"status\":\"running\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"yaw\":" + String(yaw, 2) + ",";
  json += "\"relPitch\":" + String(getRelativePitch(), 2) + ",";
  json += "\"relRoll\":" + String(getRelativeRoll(), 2) + ",";
  json += "\"relYaw\":" + String(getRelativeYaw(), 2) + ",";
  json += "\"accPitch\":" + String(accumulatedPitch, 2) + ",";
  json += "\"accRoll\":" + String(accumulatedRoll, 2) + ",";
  json += "\"accYaw\":" + String(accumulatedYaw, 2) + ",";
  json += "\"zeroSet\":" + String(zeroSet ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetZero() {
  addCORSHeaders();
  setZeroPoint();
  String response = "{\"status\":\"ok\",\"message\":\"Zero point set\"}";
  server.send(200, "application/json", response);
}

void handleResetZero() {
  addCORSHeaders();
  resetZeroPoint();
  String response = "{\"status\":\"ok\",\"message\":\"Zero point reset\"}";
  server.send(200, "application/json", response);
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>MPU6050 Sensor Data</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; }
        .data { background: white; padding: 20px; margin: 10px 0; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .value { font-size: 24px; font-weight: bold; color: #2c3e50; }
        .status { padding: 15px; margin: 10px 0; border-radius: 8px; font-weight: bold; }
        .connected { background: #d4edda; color: #155724; border: 2px solid #c3e6cb; }
        .disconnected { background: #f8d7da; color: #721c24; border: 2px solid #f5c6cb; }
        .controls { margin: 20px 0; }
        button { padding: 12px 20px; margin: 8px; border: none; border-radius: 6px; cursor: pointer; font-size: 16px; transition: all 0.3s; }
        button:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }
        .btn-primary { background: #007bff; color: white; }
        .btn-warning { background: #ffc107; color: black; }
        .btn-danger { background: #dc3545; color: white; }
        .btn-success { background: #28a745; color: white; }
        .btn-info { background: #17a2b8; color: white; }
        .zero-controls { background: #e8f5e8; padding: 20px; margin: 20px 0; border-radius: 10px; border-left: 5px solid #28a745; }
        .visualization { background: #2c3e50; color: white; padding: 25px; border-radius: 15px; margin: 20px 0; box-shadow: 0 4px 15px rgba(0,0,0,0.3); }
        .cube-container { width: 300px; height: 300px; margin: 20px auto; perspective: 1000px; }
        .cube { width: 100%; height: 100%; position: relative; transform-style: preserve-3d; transition: transform 0.1s ease-out; }
        .face { position: absolute; width: 300px; height: 300px; border: 3px solid #34495e; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: bold; color: white; background: rgba(52, 152, 219, 0.8); }
        .front { transform: rotateY(0deg) translateZ(150px); background: rgba(231, 76, 60, 0.8); }
        .back { transform: rotateY(180deg) translateZ(150px); background: rgba(52, 152, 219, 0.8); }
        .right { transform: rotateY(90deg) translateZ(150px); background: rgba(46, 204, 113, 0.8); }
        .left { transform: rotateY(-90deg) translateZ(150px); background: rgba(155, 89, 182, 0.8); }
        .top { transform: rotateX(90deg) translateZ(150px); background: rgba(241, 196, 15, 0.8); }
        .bottom { transform: rotateX(-90deg) translateZ(150px); background: rgba(230, 126, 34, 0.8); }
        .data-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 15px; }
        .data-item { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
        .data-label { font-weight: bold; color: #495057; margin-bottom: 5px; }
        h1 { color: #2c3e50; text-align: center; margin-bottom: 30px; }
        h3 { color: #343a40; margin-bottom: 15px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>MPU6050 Sensor Data</h1>
        
        <div class="status" id="status">Disconnected</div>
        
        <div class="data">
            <h3>Sensor Readings</h3>
            <div class="data-grid">
                <div class="data-item">
                    <div class="data-label">Pitch</div>
                    <div class="value" id="pitch">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Roll</div>
                    <div class="value" id="roll">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Yaw</div>
                    <div class="value" id="yaw">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Relative Pitch</div>
                    <div class="value" id="relPitch">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Relative Roll</div>
                    <div class="value" id="relRoll">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Relative Yaw</div>
                    <div class="value" id="relYaw">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Accumulated Pitch</div>
                    <div class="value" id="accPitch">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Accumulated Roll</div>
                    <div class="value" id="accRoll">0</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Accumulated Yaw</div>
                    <div class="value" id="accYaw">0</div>
                </div>
            </div>
        </div>

        <div class="zero-controls">
            <h3>Zero Point Control</h3>
            <button class="btn-success" onclick="sendCommand('SET_ZERO')">Set Zero Point</button>
            <button class="btn-warning" onclick="sendCommand('RESET_ZERO')">Reset Zero</button>
            <button class="btn-danger" onclick="sendCommand('RESET_ANGLES')">Reset All Angles</button>
            <div style="margin-top: 15px; padding: 10px; background: white; border-radius: 5px;">
                <div style="font-size: 14px; color: #666;">
                    <strong>Zero Point:</strong> <span id="zeroStatus" style="color: #dc3545; font-weight: bold;">Not Set</span>
                </div>
                <div style="font-size: 12px; color: #888; margin-top: 8px;">
                    Zero point allows you to set a reference position. Relative angles show deviation from zero point.
                    Accumulated angles show total rotation without limits (can exceed 360°).
                </div>
            </div>
        </div>

        <div class="visualization">
            <h3 style="color: white; text-align: center;">3D Platform Visualization</h3>
            <div class="cube-container">
                <div class="cube" id="cube">
                    <div class="face front">FRONT</div>
                    <div class="face back">BACK</div>
                    <div class="face right">RIGHT</div>
                    <div class="face left">LEFT</div>
                    <div class="face top">TOP</div>
                    <div class="face bottom">BOTTOM</div>
                </div>
            </div>
        </div>

        <div class="controls">
            <h3>Sensor Controls</h3>
            <button class="btn-primary" onclick="sendCommand('GET_DATA')">Get Sensor Data</button>
            <button class="btn-warning" onclick="sendCommand('RECALIBRATE')">Recalibrate</button>
        </div>

        <div class="data">
            <h3>API Information</h3>
            <div style="background: #e9ecef; padding: 15px; border-radius: 5px;">
                <p><strong>GET /api/status</strong> - Get device status</p>
                <p><strong>POST /api/setZero</strong> - Set zero point</p>
                <p><strong>POST /api/resetZero</strong> - Reset zero point</p>
            </div>
        </div>
        
        <div class="data">
            <h3>Last Message</h3>
            <div id="lastMessage" style="background: #f8f9fa; padding: 10px; border-radius: 5px; font-family: monospace; min-height: 20px;">No data received</div>
        </div>
    </div>

    <script>
        let ws = null;
        let statusDiv = document.getElementById('status');
        let lastMessageDiv = document.getElementById('lastMessage');
        let cube = document.getElementById('cube');
        let zeroStatusSpan = document.getElementById('zeroStatus');
        
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = protocol + '//' + window.location.hostname + ':81';
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                statusDiv.textContent = 'Connected';
                statusDiv.className = 'status connected';
            };
            
            ws.onmessage = function(event) {
                console.log('Received:', event.data);
                lastMessageDiv.textContent = event.data;
                
                // Parse sensor data
                if (event.data.includes('PITCH:') && event.data.includes('ROLL:') && event.data.includes('YAW:')) {
                    const data = event.data;
                    
                    // Parse all data fields
                    const pitchMatch = data.match(/PITCH:([-\d.]+)/);
                    const rollMatch = data.match(/ROLL:([-\d.]+)/);
                    const yawMatch = data.match(/YAW:([-\d.]+)/);
                    const relPitchMatch = data.match(/REL_PITCH:([-\d.]+)/);
                    const relRollMatch = data.match(/REL_ROLL:([-\d.]+)/);
                    const relYawMatch = data.match(/REL_YAW:([-\d.]+)/);
                    const accPitchMatch = data.match(/ACC_PITCH:([-\d.]+)/);
                    const accRollMatch = data.match(/ACC_ROLL:([-\d.]+)/);
                    const accYawMatch = data.match(/ACC_YAW:([-\d.]+)/);
                    const zeroSetMatch = data.match(/ZERO_SET:(true|false)/);
                    
                    if (pitchMatch) {
                        const pitch = parseFloat(pitchMatch[1]);
                        document.getElementById('pitch').textContent = pitch.toFixed(1) + '°';
                    }
                    if (rollMatch) {
                        const roll = parseFloat(rollMatch[1]);
                        document.getElementById('roll').textContent = roll.toFixed(1) + '°';
                    }
                    if (yawMatch) {
                        const yaw = parseFloat(yawMatch[1]);
                        document.getElementById('yaw').textContent = yaw.toFixed(1) + '°';
                    }
                    if (relPitchMatch) {
                        const relPitch = parseFloat(relPitchMatch[1]);
                        document.getElementById('relPitch').textContent = relPitch.toFixed(1) + '°';
                    }
                    if (relRollMatch) {
                        const relRoll = parseFloat(relRollMatch[1]);
                        document.getElementById('relRoll').textContent = relRoll.toFixed(1) + '°';
                    }
                    if (relYawMatch) {
                        const relYaw = parseFloat(relYawMatch[1]);
                        document.getElementById('relYaw').textContent = relYaw.toFixed(1) + '°';
                    }
                    if (accPitchMatch) {
                        const accPitch = parseFloat(accPitchMatch[1]);
                        document.getElementById('accPitch').textContent = accPitch.toFixed(1) + '°';
                    }
                    if (accRollMatch) {
                        const accRoll = parseFloat(accRollMatch[1]);
                        document.getElementById('accRoll').textContent = accRoll.toFixed(1) + '°';
                    }
                    if (accYawMatch) {
                        const accYaw = parseFloat(accYawMatch[1]);
                        document.getElementById('accYaw').textContent = accYaw.toFixed(1) + '°';
                    }
                    if (zeroSetMatch) {
                        const zeroSet = zeroSetMatch[1] === 'true';
                        zeroStatusSpan.textContent = zeroSet ? 'Set' : 'Not Set';
                        zeroStatusSpan.style.color = zeroSet ? '#28a745' : '#dc3545';
                    }
                    
                    // Update 3D visualization
                    update3DVisualization(pitchMatch[1], rollMatch[1], yawMatch[1]);
                }
                
                // Parse zero point messages
                if (event.data === 'ZERO_POINT_SET') {
                    zeroStatusSpan.textContent = 'Set';
                    zeroStatusSpan.style.color = '#28a745';
                    showNotification('Zero point set successfully', 'success');
                }
                if (event.data === 'ZERO_POINT_RESET') {
                    zeroStatusSpan.textContent = 'Not Set';
                    zeroStatusSpan.style.color = '#dc3545';
                    showNotification('Zero point reset', 'info');
                }
                if (event.data.startsWith('ZERO_SET:')) {
                    zeroStatusSpan.textContent = 'Set';
                    zeroStatusSpan.style.color = '#28a745';
                }
                if (event.data === 'ZERO_RESET') {
                    zeroStatusSpan.textContent = 'Not Set';
                    zeroStatusSpan.style.color = '#dc3545';
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                statusDiv.textContent = 'Disconnected';
                statusDiv.className = 'status disconnected';
                
                setTimeout(connectWebSocket, 2000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function update3DVisualization(pitch, roll, yaw) {
            // Apply rotation to the 3D cube
            cube.style.transform = `rotateX(${roll}deg) rotateY(${yaw}deg) rotateZ(${pitch}deg)`;
        }
        
        function sendCommand(command) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(command);
                console.log('Sent command:', command);
            } else {
                console.log('WebSocket not connected');
                showNotification('WebSocket not connected', 'error');
            }
        }
        
        function showNotification(message, type = 'info') {
            // Create notification element
            const notification = document.createElement('div');
            notification.textContent = message;
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                padding: 15px 20px;
                border-radius: 5px;
                color: white;
                font-weight: bold;
                z-index: 1000;
                background: ${type === 'success' ? '#28a745' : type === 'error' ? '#dc3545' : '#17a2b8'};
                box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            `;
            
            document.body.appendChild(notification);
            
            // Remove after 3 seconds
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 3000);
        }

        // Initialize when page loads
        window.addEventListener('load', function() {
            connectWebSocket();
        });
    </script>
</body>
</html>
)rawliteral";

  addCORSHeaders();
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  WiFi.hostname(newHostname.c_str());
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  
  calibrateSensor();
  
  server.on("/", handleRoot);
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/setZero", HTTP_POST, handleSetZero);
  server.on("/api/resetZero", HTTP_POST, handleResetZero);
  
  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/setZero", HTTP_OPTIONS, handleOptions);
  server.on("/api/resetZero", HTTP_OPTIONS, handleOptions);
  
  server.enableCORS(true);
  server.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("HTTP server started on port 80");
  Serial.println("WebSocket server started on port 81");
  
  lastDataSend = millis();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  if (!calibrated) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (lastTime == 0) deltaTime = 0.01;
  lastTime = currentTime;
  
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;
  
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  pitch += gyroX * deltaTime * 180.0 / PI;
  roll += gyroY * deltaTime * 180.0 / PI;
  yaw += gyroZ * deltaTime * 180.0 / PI;
  
  float alpha = 0.96;
  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll = alpha * roll + (1.0 - alpha) * accelRoll;
  
  if (clientConnected && (currentTime - lastDataSend >= SEND_INTERVAL)) {
    if (dataChanged() || lastDataSend == 0) {
      sendSensorData();
      lastDataSend = currentTime;
    }
  }
  
  delay(10);
}
