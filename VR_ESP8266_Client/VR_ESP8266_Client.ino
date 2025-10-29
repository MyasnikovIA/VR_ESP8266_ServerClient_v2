#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>

Adafruit_MPU6050 mpu;

// Настройки WiFi по умолчанию
const char* ap_ssid = "VR_Head_Hom_001";
const char* ap_password = "12345678";

// Создание веб-сервера на порту 80
ESP8266WebServer server(80);

// WebSocket сервер на порту 81 для всех клиентов
WebSocketsServer webSocket = WebSocketsServer(81);

// DHCP сервер
WiFiUDP udp;
const unsigned int DHCP_SERVER_PORT = 67;
const unsigned int DHCP_CLIENT_PORT = 68;

// DHCP структуры
struct DHCPMessage {
    uint8_t op;      // Message op code / message type.
    uint8_t htype;   // Hardware address type
    uint8_t hlen;    // Hardware address length
    uint8_t hops;    // Hops
    uint32_t xid;    // Transaction ID
    uint16_t secs;   // Seconds
    uint16_t flags;  // Flags
    uint32_t ciaddr; // Client IP address
    uint32_t yiaddr; // Your (client) IP address
    uint32_t siaddr; // Next server IP address
    uint32_t giaddr; // Relay agent IP address
    uint8_t chaddr[16]; // Client hardware address
    uint8_t sname[64];  // Server host name
    uint8_t file[128];  // Boot file name
    uint8_t options[312]; // Optional parameters
};

// Структура для хранения информации об устройстве
struct DeviceInfo {
  char ip[16];
  char mac[18];
  char hostname[32];
  int rssi;
  bool connected;
  unsigned long lastSeen;
};

// Структура для настроек сети
struct NetworkSettings {
  char ssid[32];
  char password[32];
  char subnet[4];
  bool configured;
  bool apModeEnabled;
  char sta_ssid[32];
  char sta_password[32];
  char device_comment[256];
};

// Sensor data - БЕСКОНЕЧНЫЕ УГЛЫ
float pitch = 0, roll = 0, yaw = 0; // Теперь бесконечные углы
float lastSentPitch = 0, lastSentRoll = 0, lastSentYaw = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
bool calibrated = false;
unsigned long lastTime = 0;

// Относительный ноль
float zeroPitchOffset = 0, zeroRollOffset = 0, zeroYawOffset = 0;
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

// Массив для хранения подключенных устройств
const int MAX_DEVICES = 10;
DeviceInfo devices[MAX_DEVICES];
int deviceCount = 0;

// Настройки сети
NetworkSettings networkSettings;

// DHCP пул адресов
IPAddress dhcpStartIP;
IPAddress dhcpEndIP;
const int DHCP_POOL_SIZE = 20;
bool dhcpLeases[DHCP_POOL_SIZE];
String dhcpMacTable[DHCP_POOL_SIZE];

// Время последнего сканирования
unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 10000;

// Время последней записи в EEPROM
unsigned long lastEEPROMSave = 0;
const unsigned long EEPROM_SAVE_INTERVAL = 5000;
bool eepromDirty = false;

// Переменные для сканирования WiFi
String wifiNetworks = "";
unsigned long lastWifiScan = 0;
const unsigned long WIFI_SCAN_INTERVAL = 30000;
bool scanningWifi = false;

// Переменные для регистрации
bool registrationAttempted = false;

// Прототипы функций для DHCP
void handleDHCPDiscover(DHCPMessage& discoverMsg);
void handleDHCPRequest(DHCPMessage& requestMsg);
void sendDHCPOffer(DHCPMessage& discoverMsg, int ipIndex);
void sendDHCPAck(DHCPMessage& requestMsg, const String& clientMAC);
String macToString(uint8_t* mac);
int getIPFromMAC(const String& mac);
void attemptRegistration();

// Безопасное копирование строк с проверкой границ
void safeStrcpy(char* dest, const char* src, size_t destSize) {
  if (dest == NULL || src == NULL || destSize == 0) return;
  
  size_t srcLen = strlen(src);
  if (srcLen >= destSize) {
    srcLen = destSize - 1;
  }
  
  strncpy(dest, src, srcLen);
  dest[srcLen] = '\0';
}

// Функция для поиска устройства в массиве
int findDeviceByMAC(const char* mac) {
  if (mac == NULL) return -1;
  
  for (int i = 0; i < deviceCount; i++) {
    if (strcmp(devices[i].mac, mac) == 0) {
      return i;
    }
  }
  return -1;
}

// Функция для добавления нового устройства
bool addDevice(const char* ip, const char* mac, const char* hostname, int rssi) {
  if (ip == NULL || mac == NULL) return false;
  
  int index = findDeviceByMAC(mac);
  
  if (index == -1) {
    if (deviceCount < MAX_DEVICES) {
      safeStrcpy(devices[deviceCount].ip, ip, sizeof(devices[deviceCount].ip));
      safeStrcpy(devices[deviceCount].mac, mac, sizeof(devices[deviceCount].mac));
      
      if (hostname && strlen(hostname) > 0) {
        safeStrcpy(devices[deviceCount].hostname, hostname, sizeof(devices[deviceCount].hostname));
      } else {
        safeStrcpy(devices[deviceCount].hostname, "Unknown", sizeof(devices[deviceCount].hostname));
      }
      
      devices[deviceCount].rssi = rssi;
      devices[deviceCount].connected = true;
      devices[deviceCount].lastSeen = millis();
      deviceCount++;
      
      Serial.printf("New device: %s (%s) - %s - RSSI: %d\n", 
                   hostname ? hostname : "Unknown", ip, mac, rssi);
      return true;
    } else {
      Serial.println("Device limit reached!");
      return false;
    }
  } else {
    devices[index].rssi = rssi;
    safeStrcpy(devices[index].ip, ip, sizeof(devices[index].ip));
    devices[index].connected = true;
    devices[index].lastSeen = millis();
    if (hostname && strlen(hostname) > 0) {
      safeStrcpy(devices[index].hostname, hostname, sizeof(devices[index].hostname));
    }
    return true;
  }
}

// Функция для отправки полного списка устройств веб-клиентам
void sendDevicesListToWebClients() {
  DynamicJsonDocument doc(4096);
  
  JsonArray devicesArray = doc.createNestedArray("devices");
  
  // Подсчитываем статистику
  int onlineCount = 0;
  
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].connected) onlineCount++;
    
    JsonObject deviceObj = devicesArray.createNestedObject();
    deviceObj["ip"] = devices[i].ip;
    deviceObj["mac"] = devices[i].mac;
    deviceObj["hostname"] = devices[i].hostname;
    deviceObj["rssi"] = devices[i].rssi;
    deviceObj["connected"] = devices[i].connected;
  }
  
  doc["type"] = "devices_list";
  doc["totalDevices"] = deviceCount;
  doc["onlineDevices"] = onlineCount;
  doc["espIp"] = WiFi.softAPIP().toString();
  doc["wifiStatus"] = WiFi.status() == WL_CONNECTED ? "connected" : "disconnected";
  doc["apModeEnabled"] = networkSettings.apModeEnabled;
  
  String json;
  if (serializeJson(doc, json) == 0) {
    Serial.println("Error: Failed to serialize JSON for devices list");
  } else {
    webSocket.broadcastTXT(json);
  }
}

// Функция сканирования сети
void scanNetwork() {
  Serial.println("Starting network scan...");
  
  // Помечаем все устройства как отключенные
  for (int i = 0; i < deviceCount; i++) {
    devices[i].connected = false;
  }
  
  // Проверяем активные подключения к точке доступа
  struct station_info *station = wifi_softap_get_station_info();
  while (station != NULL) {
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             station->bssid[0], station->bssid[1], station->bssid[2],
             station->bssid[3], station->bssid[4], station->bssid[5]);
    
    char ip[16];
    IPAddress ipAddr = IPAddress(station->ip);
    snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
    
    // Добавляем устройство только если это не дубликат (по MAC-адресу)
    if (findDeviceByMAC(mac) == -1) {
      addDevice(ip, mac, "WiFi Client", -50);
    }
    
    station = STAILQ_NEXT(station, next);
  }
  wifi_softap_free_station_info();
  
  // Помечаем устройства как отключенные если не видели их больше минуты
  unsigned long currentTime = millis();
  for (int i = 0; i < deviceCount; i++) {
    if ((currentTime - devices[i].lastSeen) > 60000) {
      devices[i].connected = false;
    }
  }
  
  // Отправляем обновленный список устройств веб-клиентам
  sendDevicesListToWebClients();
  
  Serial.printf("Scan complete. Found %d devices\n", deviceCount);
  lastScanTime = millis();
}

// Функция сканирования WiFi сетей
void scanWiFiNetworks() {
  if (scanningWifi) return;
  
  scanningWifi = true;
  Serial.println("Scanning WiFi networks...");
  
  int n = WiFi.scanNetworks();
  wifiNetworks = "";
  
  DynamicJsonDocument doc(4096);
  JsonArray networks = doc.to<JsonArray>();
  
  for (int i = 0; i < n; ++i) {
    JsonObject network = networks.createNestedObject();
    network["ssid"] = WiFi.SSID(i);
    network["rssi"] = WiFi.RSSI(i);
    network["encryption"] = (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? "open" : "secured";
  }
  
  serializeJson(doc, wifiNetworks);
  WiFi.scanDelete();
  
  Serial.printf("Found %d WiFi networks\n", n);
  scanningWifi = false;
  lastWifiScan = millis();
}

// Инициализация DHCP сервера
void initDHCPServer() {
  int subnet = atoi(networkSettings.subnet);
  if (subnet < 1 || subnet > 254) {
    subnet = 50;
  }
  
  dhcpStartIP = IPAddress(192, 168, subnet, 2);
  dhcpEndIP = IPAddress(192, 168, subnet, 20);
  
  // Инициализация пула адресов
  for (int i = 0; i < DHCP_POOL_SIZE; i++) {
    dhcpLeases[i] = false;
    dhcpMacTable[i] = "";
  }
  
  // Запуск UDP сервера для DHCP
  if (udp.begin(DHCP_SERVER_PORT)) {
    Serial.println("DHCP Server started on port 67");
  } else {
    Serial.println("Failed to start DHCP server");
  }
}

// Обработка DHCP запросов
void handleDHCP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    if (packetSize >= sizeof(DHCPMessage)) {
      Serial.println("DHCP packet too large");
      return;
    }
    
    DHCPMessage dhcpMsg;
    udp.read((uint8_t*)&dhcpMsg, sizeof(DHCPMessage));
    
    // Определяем тип DHCP сообщения
    uint8_t messageType = 0;
    for (int i = 0; i < 312; i++) {
      if (dhcpMsg.options[i] == 53) { // DHCP Message Type
        messageType = dhcpMsg.options[i + 2];
        break;
      }
    }
    
    if (messageType == 1) { // DHCP Discover
      handleDHCPDiscover(dhcpMsg);
    } else if (messageType == 3) { // DHCP Request
      handleDHCPRequest(dhcpMsg);
    }
  }
}

// Обработка DHCP Discover
void handleDHCPDiscover(DHCPMessage& discoverMsg) {
  Serial.println("DHCP Discover received");
  
  // Ищем свободный IP в пуле
  int freeIPIndex = -1;
  String clientMAC = macToString(discoverMsg.chaddr);
  
  // Сначала проверяем, есть ли уже аренда для этого MAC
  for (int i = 0; i < DHCP_POOL_SIZE; i++) {
    if (dhcpMacTable[i] == clientMAC) {
      freeIPIndex = i;
      break;
    }
  }
  
  // Если нет, ищем свободный IP
  if (freeIPIndex == -1) {
    for (int i = 0; i < DHCP_POOL_SIZE; i++) {
      if (!dhcpLeases[i]) {
        freeIPIndex = i;
        break;
      }
    }
  }
  
  if (freeIPIndex != -1) {
    // Отправляем DHCP Offer
    sendDHCPOffer(discoverMsg, freeIPIndex);
  } else {
    Serial.println("No free IP addresses in DHCP pool");
  }
}

// Обработка DHCP Request
void handleDHCPRequest(DHCPMessage& requestMsg) {
  Serial.println("DHCP Request received");
  
  String clientMAC = macToString(requestMsg.chaddr);
  
  // Отправляем DHCP Ack
  sendDHCPAck(requestMsg, clientMAC);
  
  // Добавляем устройство в список
  int subnet = atoi(networkSettings.subnet);
  int clientIPNum = getIPFromMAC(clientMAC);
  
  if (clientIPNum != -1) {
    IPAddress clientIP = IPAddress(192, 168, subnet, clientIPNum);
    addDevice(clientIP.toString().c_str(), clientMAC.c_str(), "DHCP Client", -50);
  }
}

// Отправка DHCP Offer
void sendDHCPOffer(DHCPMessage& discoverMsg, int ipIndex) {
  DHCPMessage offerMsg;
  memset(&offerMsg, 0, sizeof(DHCPMessage));
  
  offerMsg.op = 2; // BOOTREPLY
  offerMsg.htype = 1; // Ethernet
  offerMsg.hlen = 6;
  offerMsg.xid = discoverMsg.xid;
  
  int subnet = atoi(networkSettings.subnet);
  offerMsg.yiaddr = IPAddress(192, 168, subnet, ipIndex + 2).v4();
  offerMsg.siaddr = WiFi.softAPIP().v4();
  
  memcpy(offerMsg.chaddr, discoverMsg.chaddr, 16);
  strcpy((char*)offerMsg.sname, "VR_Head_DHCP");
  
  // Добавляем опции
  int optIndex = 0;
  offerMsg.options[optIndex++] = 53; // DHCP Message Type
  offerMsg.options[optIndex++] = 1;
  offerMsg.options[optIndex++] = 2; // Offer
  
  offerMsg.options[optIndex++] = 1; // Subnet Mask
  offerMsg.options[optIndex++] = 4;
  uint32_t subnetMask = IPAddress(255, 255, 255, 0).v4();
  memcpy(&offerMsg.options[optIndex], &subnetMask, 4);
  optIndex += 4;
  
  offerMsg.options[optIndex++] = 3; // Router
  offerMsg.options[optIndex++] = 4;
  uint32_t router = WiFi.softAPIP().v4();
  memcpy(&offerMsg.options[optIndex], &router, 4);
  optIndex += 4;
  
  offerMsg.options[optIndex++] = 51; // IP Lease Time
  offerMsg.options[optIndex++] = 4;
  uint32_t leaseTime = 3600; // 1 hour
  memcpy(&offerMsg.options[optIndex], &leaseTime, 4);
  optIndex += 4;
  
  offerMsg.options[optIndex++] = 54; // DHCP Server Identifier
  offerMsg.options[optIndex++] = 4;
  memcpy(&offerMsg.options[optIndex], &router, 4);
  optIndex += 4;
  
  offerMsg.options[optIndex++] = 255; // End option
  
  // Отправка пакета
  udp.beginPacket(IPAddress(255, 255, 255, 255), DHCP_CLIENT_PORT);
  udp.write((uint8_t*)&offerMsg, sizeof(DHCPMessage));
  udp.endPacket();
  
  // Резервируем IP
  dhcpLeases[ipIndex] = true;
  dhcpMacTable[ipIndex] = macToString(discoverMsg.chaddr);
  
  Serial.printf("DHCP Offer sent for IP: 192.168.%d.%d\n", subnet, ipIndex + 2);
}

// Отправка DHCP Ack
void sendDHCPAck(DHCPMessage& requestMsg, const String& clientMAC) {
  DHCPMessage ackMsg;
  memset(&ackMsg, 0, sizeof(DHCPMessage));
  
  ackMsg.op = 2; // BOOTREPLY
  ackMsg.htype = 1;
  ackMsg.hlen = 6;
  ackMsg.xid = requestMsg.xid;
  
  int subnet = atoi(networkSettings.subnet);
  int clientIP = getIPFromMAC(clientMAC);
  
  if (clientIP == -1) {
    Serial.println("Error: No IP available for client");
    return;
  }
  
  ackMsg.yiaddr = IPAddress(192, 168, subnet, clientIP).v4();
  ackMsg.siaddr = WiFi.softAPIP().v4();
  
  memcpy(ackMsg.chaddr, requestMsg.chaddr, 16);
  strcpy((char*)ackMsg.sname, "VR_Head_DHCP");
  
  // Добавляем опции
  int optIndex = 0;
  ackMsg.options[optIndex++] = 53; // DHCP Message Type
  ackMsg.options[optIndex++] = 1;
  ackMsg.options[optIndex++] = 5; // ACK
  
  ackMsg.options[optIndex++] = 1; // Subnet Mask
  ackMsg.options[optIndex++] = 4;
  uint32_t subnetMask = IPAddress(255, 255, 255, 0).v4();
  memcpy(&ackMsg.options[optIndex], &subnetMask, 4);
  optIndex += 4;
  
  ackMsg.options[optIndex++] = 3; // Router
  ackMsg.options[optIndex++] = 4;
  uint32_t router = WiFi.softAPIP().v4();
  memcpy(&ackMsg.options[optIndex], &router, 4);
  optIndex += 4;
  
  ackMsg.options[optIndex++] = 51; // IP Lease Time
  ackMsg.options[optIndex++] = 4;
  uint32_t leaseTime = 3600;
  memcpy(&ackMsg.options[optIndex], &leaseTime, 4);
  optIndex += 4;
  
  ackMsg.options[optIndex++] = 54; // DHCP Server Identifier
  ackMsg.options[optIndex++] = 4;
  memcpy(&ackMsg.options[optIndex], &router, 4);
  optIndex += 4;
  
  ackMsg.options[optIndex++] = 255; // End option
  
  udp.beginPacket(IPAddress(255, 255, 255, 255), DHCP_CLIENT_PORT);
  udp.write((uint8_t*)&ackMsg, sizeof(DHCPMessage));
  udp.endPacket();
  
  Serial.printf("DHCP ACK sent for IP: 192.168.%d.%d to MAC: %s\n", subnet, clientIP, clientMAC.c_str());
}

// Вспомогательная функция для преобразования MAC в строку
String macToString(uint8_t* mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// Получение IP из MAC
int getIPFromMAC(const String& mac) {
  for (int i = 0; i < DHCP_POOL_SIZE; i++) {
    if (dhcpMacTable[i] == mac) {
      return i + 2;
    }
  }
  
  // Если IP не найден, назначаем новый
  for (int i = 0; i < DHCP_POOL_SIZE; i++) {
    if (!dhcpLeases[i]) {
      dhcpLeases[i] = true;
      dhcpMacTable[i] = mac;
      return i + 2;
    }
  }
  
  return -1; // Нет свободных IP
}

// Функция для обнуления всех углов
void resetAllAngles() {
  pitch = 0;
  roll = 0;
  yaw = 0;
  lastSentPitch = 0;
  lastSentRoll = 0;
  lastSentYaw = 0;
  accumulatedPitch = 0;
  accumulatedRoll = 0;
  accumulatedYaw = 0;
  prevPitch = 0;
  prevRoll = 0;
  prevYaw = 0;
  firstMeasurement = true;
  
  webSocket.broadcastTXT("All angles reset to zero");
  Serial.println("All angles reset to zero");
}

// Функция для установки текущего положения как нулевой точки
void setCurrentPositionAsZero() {
  zeroPitchOffset = accumulatedPitch;
  zeroRollOffset = accumulatedRoll;
  zeroYawOffset = accumulatedYaw;
  zeroSet = true;
  
  Serial.println("Current position set as zero point");
  Serial.print("Zero Pitch Offset: "); Serial.print(zeroPitchOffset, 2);
  Serial.print(" Roll Offset: "); Serial.print(zeroRollOffset, 2);
  Serial.print(" Yaw Offset: "); Serial.println(zeroYawOffset, 2);
  
  String message = "ZERO_SET:PITCH:" + String(zeroPitchOffset, 2) + 
                   ",ROLL:" + String(zeroRollOffset, 2) + 
                   ",YAW:" + String(zeroYawOffset, 2);
  webSocket.broadcastTXT(message);
}

// Обнуление точек поворота
void resetZeroOffsets() {
  zeroPitchOffset = 0;
  zeroRollOffset = 0;
  zeroYawOffset = 0;
  zeroSet = false;
  
  webSocket.broadcastTXT("Zero offsets reset to zero");
  Serial.println("Zero offsets reset to zero");
}

// Сброс относительного нуля
void resetZeroPoint() {
  zeroPitchOffset = 0;
  zeroRollOffset = 0;
  zeroYawOffset = 0;
  zeroSet = false;
  
  webSocket.broadcastTXT("Zero point reset");
  Serial.println("Zero point reset");
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
  
  // Вычисляем разницу углов с учетом переходов через 180/-180 для ВСЕХ осей
  float deltaPitch = pitch - prevPitch;
  float deltaRoll = roll - prevRoll;
  float deltaYaw = yaw - prevYaw;
  
  // Корректируем разницу для переходов через границу ±180 для ВСЕХ осей
  if (deltaPitch > 180) deltaPitch -= 360;
  else if (deltaPitch < -180) deltaPitch += 360;
  
  if (deltaRoll > 180) deltaRoll -= 360;
  else if (deltaRoll < -180) deltaRoll += 360;
  
  if (deltaYaw > 180) deltaYaw -= 360;
  else if (deltaYaw < -180) deltaYaw += 360;
  
  // Накопление углов для ВСЕХ осей
  accumulatedPitch += deltaPitch;
  accumulatedRoll += deltaRoll;
  accumulatedYaw += deltaYaw;
  
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
}

// Получение относительных углов (относительно зафиксированного нуля)
double getRelativePitch() {
  if (!zeroSet) return accumulatedPitch;
  return accumulatedPitch - zeroPitchOffset;
}

double getRelativeRoll() {
  if (!zeroSet) return accumulatedRoll;
  return accumulatedRoll - zeroRollOffset;
}

double getRelativeYaw() {
  if (!zeroSet) return accumulatedYaw;
  return accumulatedYaw - zeroYawOffset;
}

void calibrateSensor() {
  Serial.println("Calibrating MPU6050...");
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
  Serial.print("Gyro offsets - X: "); Serial.print(gyroOffsetX, 6);
  Serial.print(" Y: "); Serial.print(gyroOffsetY, 6);
  Serial.print(" Z: "); Serial.println(gyroOffsetZ, 6);
}

// Функция проверки изменения данных
bool dataChanged() {
  return (abs(pitch - lastSentPitch) >= CHANGE_THRESHOLD ||
          abs(roll - lastSentRoll) >= CHANGE_THRESHOLD ||
          abs(yaw - lastSentYaw) >= CHANGE_THRESHOLD);
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
                ",ZERO_SET:" + String(zeroSet ? "true" : "false") +
                ",ZERO_PITCH:" + String(zeroPitchOffset, 2) +
                ",ZERO_ROLL:" + String(zeroRollOffset, 2) +
                ",ZERO_YAW:" + String(zeroYawOffset, 2);
  
  webSocket.broadcastTXT(data);
  lastSentPitch = pitch;
  lastSentRoll = roll;
  lastSentYaw = yaw;
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
        
        // Отправляем список устройств при подключении
        sendDevicesListToWebClients();
      }
      break;
      
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        Serial.printf("[%u] Received: %s\n", num, message);
        
        if (message == "GET_DATA") {
          sendSensorData();
        }
        else if (message == "RECALIBRATE" || message == "CAL") {
          calibrated = false;
          calibrateSensor();
          String calMessage = "RECALIBRATION_COMPLETE";
          webSocket.broadcastTXT(calMessage);
        }
        else if (message == "RESET_ANGLES" || message == "RA") {
          resetAllAngles();
          sendSensorData();
        }
        else if (message == "SET_ZERO" || message == "SZ") {
          setCurrentPositionAsZero();
          webSocket.broadcastTXT("ZERO_POINT_SET");
        }
        else if (message == "RESET_ZERO" || message == "RZ") {
          resetZeroPoint();
          webSocket.broadcastTXT("ZERO_POINT_RESET");
        }
        else if (message == "RESET_OFFSETS" || message == "RO") {
          resetZeroOffsets();
          webSocket.broadcastTXT("ZERO_OFFSETS_RESET");
        }
        else if (message == "SCAN_NETWORK") {
          scanNetwork();
        }
        else if (message == "SCAN_WIFI") {
          scanWiFiNetworks();
          String response = "{\"type\":\"wifi_networks\",\"networks\":" + wifiNetworks + "}";
          webSocket.sendTXT(num, response);
        }
        else if (message == "HELP" || message == "?") {
          String help = "Available commands: ";
          help += "GET_DATA, RECALIBRATE(CAL), RESET_ANGLES(RA), ";
          help += "SET_ZERO(SZ), RESET_ZERO(RZ), RESET_OFFSETS(RO), ";
          help += "SCAN_NETWORK, SCAN_WIFI, HELP(?)";
          webSocket.sendTXT(num, help);
        }
        else {
          // Обработка JSON сообщений для сетевых функций
          DynamicJsonDocument doc(512);
          DeserializationError error = deserializeJson(doc, payload);
          
          if (!error) {
            String messageType = doc["type"];
            
            if (messageType == "get_devices") {
              sendDevicesListToWebClients();
            } else if (messageType == "get_settings") {
              DynamicJsonDocument settingsDoc(512);
              settingsDoc["type"] = "current_settings";
              JsonObject settings = settingsDoc.createNestedObject("settings");
              settings["ssid"] = networkSettings.ssid;
              settings["password"] = networkSettings.password;
              settings["subnet"] = networkSettings.subnet;
              settings["sta_ssid"] = networkSettings.sta_ssid;
              settings["sta_password"] = networkSettings.sta_password;
              settings["device_comment"] = networkSettings.device_comment;
              settings["ap_mode_enabled"] = networkSettings.apModeEnabled;
              
              String json;
              serializeJson(settingsDoc, json);
              webSocket.sendTXT(num, json);
            } else if (messageType == "update_network_settings") {
              JsonObject settings = doc["settings"];
              
              if (settings.containsKey("ssid")) {
                safeStrcpy(networkSettings.ssid, settings["ssid"], sizeof(networkSettings.ssid));
              }
              if (settings.containsKey("password")) {
                safeStrcpy(networkSettings.password, settings["password"], sizeof(networkSettings.password));
              }
              if (settings.containsKey("subnet")) {
                safeStrcpy(networkSettings.subnet, settings["subnet"], sizeof(networkSettings.subnet));
              }
              if (settings.containsKey("sta_ssid")) {
                safeStrcpy(networkSettings.sta_ssid, settings["sta_ssid"], sizeof(networkSettings.sta_ssid));
              }
              if (settings.containsKey("sta_password")) {
                safeStrcpy(networkSettings.sta_password, settings["sta_password"], sizeof(networkSettings.sta_password));
              }
              if (settings.containsKey("device_comment")) {
                safeStrcpy(networkSettings.device_comment, settings["device_comment"], sizeof(networkSettings.device_comment));
              }
              if (settings.containsKey("ap_mode_enabled")) {
                networkSettings.apModeEnabled = settings["ap_mode_enabled"];
              }
              
              networkSettings.configured = true;
              saveNetworkSettingsToEEPROM();
              eepromDirty = true;
              
              webSocket.sendTXT(num, "{\"type\":\"settings_updated\",\"status\":\"success\"}");
            }
          }
        }
      }
      break;
  }
}

// Функция для сохранения настроек сети в EEPROM
void saveNetworkSettingsToEEPROM() {
  EEPROM.begin(2048);
  
  int addr = 0;
  EEPROM.write(addr++, networkSettings.configured ? 1 : 0);
  EEPROM.write(addr++, networkSettings.apModeEnabled ? 1 : 0);
  
  // Сохраняем AP SSID
  size_t ssidLen = strlen(networkSettings.ssid);
  if (ssidLen > 31) ssidLen = 31;
  EEPROM.write(addr++, ssidLen);
  for (size_t j = 0; j < ssidLen; j++) {
    EEPROM.write(addr++, networkSettings.ssid[j]);
  }
  
  // Сохраняем AP пароль
  size_t passwordLen = strlen(networkSettings.password);
  if (passwordLen > 31) passwordLen = 31;
  EEPROM.write(addr++, passwordLen);
  for (size_t j = 0; j < passwordLen; j++) {
    EEPROM.write(addr++, networkSettings.password[j]);
  }
  
  // Сохраняем подсеть
  size_t subnetLen = strlen(networkSettings.subnet);
  if (subnetLen > 3) subnetLen = 3;
  EEPROM.write(addr++, subnetLen);
  for (size_t j = 0; j < subnetLen; j++) {
    EEPROM.write(addr++, networkSettings.subnet[j]);
  }
  
  // Сохраняем STA SSID
  size_t sta_ssidLen = strlen(networkSettings.sta_ssid);
  if (sta_ssidLen > 31) sta_ssidLen = 31;
  EEPROM.write(addr++, sta_ssidLen);
  for (size_t j = 0; j < sta_ssidLen; j++) {
    EEPROM.write(addr++, networkSettings.sta_ssid[j]);
  }
  
  // Сохраняем STA пароль
  size_t sta_passwordLen = strlen(networkSettings.sta_password);
  if (sta_passwordLen > 31) sta_passwordLen = 31;
  EEPROM.write(addr++, sta_passwordLen);
  for (size_t j = 0; j < sta_passwordLen; j++) {
    EEPROM.write(addr++, networkSettings.sta_password[j]);
  }
  
  // Сохраняем комментарий устройства
  size_t commentLen = strlen(networkSettings.device_comment);
  if (commentLen > 255) commentLen = 255;
  EEPROM.write(addr++, commentLen);
  for (size_t j = 0; j < commentLen; j++) {
    EEPROM.write(addr++, networkSettings.device_comment[j]);
  }
  
  if (EEPROM.commit()) {
    Serial.println("Network settings saved to EEPROM");
  } else {
    Serial.println("Error saving network settings to EEPROM");
  }
  EEPROM.end();
}

// Функция для загрузки настроек сети из EEPROM
void loadNetworkSettingsFromEEPROM() {
  EEPROM.begin(2048);
  
  int addr = 0;
  networkSettings.configured = (EEPROM.read(addr++) == 1);
  networkSettings.apModeEnabled = (EEPROM.read(addr++) == 1);
  
  if (networkSettings.configured) {
    // Загружаем AP SSID
    int ssidLen = EEPROM.read(addr++);
    if (ssidLen > 31) ssidLen = 31;
    for (int j = 0; j < ssidLen; j++) {
      networkSettings.ssid[j] = EEPROM.read(addr++);
    }
    networkSettings.ssid[ssidLen] = '\0';
    
    // Загружаем AP пароль
    int passwordLen = EEPROM.read(addr++);
    if (passwordLen > 31) passwordLen = 31;
    for (int j = 0; j < passwordLen; j++) {
      networkSettings.password[j] = EEPROM.read(addr++);
    }
    networkSettings.password[passwordLen] = '\0';
    
    // Загружаем подсеть
    int subnetLen = EEPROM.read(addr++);
    if (subnetLen > 3) subnetLen = 3;
    for (int j = 0; j < subnetLen; j++) {
      networkSettings.subnet[j] = EEPROM.read(addr++);
    }
    networkSettings.subnet[subnetLen] = '\0';
    
    // Загружаем STA SSID
    int sta_ssidLen = EEPROM.read(addr++);
    if (sta_ssidLen > 31) sta_ssidLen = 31;
    for (int j = 0; j < sta_ssidLen; j++) {
      networkSettings.sta_ssid[j] = EEPROM.read(addr++);
    }
    networkSettings.sta_ssid[sta_ssidLen] = '\0';
    
    // Загружаем STA пароль
    int sta_passwordLen = EEPROM.read(addr++);
    if (sta_passwordLen > 31) sta_passwordLen = 31;
    for (int j = 0; j < sta_passwordLen; j++) {
      networkSettings.sta_password[j] = EEPROM.read(addr++);
    }
    networkSettings.sta_password[sta_passwordLen] = '\0';
    
    // Загружаем комментарий устройства
    int commentLen = EEPROM.read(addr++);
    if (commentLen > 255) commentLen = 255;
    for (int j = 0; j < commentLen; j++) {
      networkSettings.device_comment[j] = EEPROM.read(addr++);
    }
    networkSettings.device_comment[commentLen] = '\0';
  } else {
    // Значения по умолчанию
    safeStrcpy(networkSettings.ssid, ap_ssid, sizeof(networkSettings.ssid));
    safeStrcpy(networkSettings.password, ap_password, sizeof(networkSettings.password));
    safeStrcpy(networkSettings.subnet, "50", sizeof(networkSettings.subnet));
    safeStrcpy(networkSettings.sta_ssid, "", sizeof(networkSettings.sta_ssid));
    safeStrcpy(networkSettings.sta_password, "", sizeof(networkSettings.sta_password));
    safeStrcpy(networkSettings.device_comment, "", sizeof(networkSettings.device_comment));
    networkSettings.apModeEnabled = true;
  }
  
  EEPROM.end();
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
  
  // Обновляем накопленные углы перед отправкой
  updateAccumulatedAngles();
  
  String json = "{";
  json += "\"status\":\"running\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"yaw\":" + String(yaw, 2) + ",";
  json += "\"relPitch\":" + String(getRelativePitch(), 2) + ",";
  json += "\"relRoll\":" + String(getRelativeRoll(), 2) + ",";
  json += "\"relYaw\":" + String(getRelativeYaw(), 2) + ",";
  json += "\"accPitch\":" + String(accumulatedPitch, 2) + ",";
  json += "\"accRoll\":" + String(accumulatedRoll, 2) + ",";
  json += "\"accYaw\":" + String(accumulatedYaw, 2) + ",";
  json += "\"zeroSet\":" + String(zeroSet ? "true" : "false") + ",";
  json += "\"zeroPitch\":" + String(zeroPitchOffset, 2) + ",";
  json += "\"zeroRoll\":" + String(zeroRollOffset, 2) + ",";
  json += "\"zeroYaw\":" + String(zeroYawOffset, 2) + ",";
  json += "\"apModeEnabled\":" + String(networkSettings.apModeEnabled ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetZero() {
  addCORSHeaders();
  setCurrentPositionAsZero();
  String response = "{\"status\":\"ok\",\"message\":\"Zero point set\"}";
  server.send(200, "application/json", response);
}

void handleResetAngles() {
  addCORSHeaders();
  resetAllAngles();
  String response = "{\"status\":\"ok\",\"message\":\"All angles reset\"}";
  server.send(200, "application/json", response);
}

void handleNetworkSettings() {
  addCORSHeaders();
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, body);
    
    if (!error) {
      JsonObject settings = doc["settings"];
      
      if (settings.containsKey("ssid")) {
        safeStrcpy(networkSettings.ssid, settings["ssid"], sizeof(networkSettings.ssid));
      }
      if (settings.containsKey("password")) {
        safeStrcpy(networkSettings.password, settings["password"], sizeof(networkSettings.password));
      }
      if (settings.containsKey("subnet")) {
        safeStrcpy(networkSettings.subnet, settings["subnet"], sizeof(networkSettings.subnet));
      }
      if (settings.containsKey("sta_ssid")) {
        safeStrcpy(networkSettings.sta_ssid, settings["sta_ssid"], sizeof(networkSettings.sta_ssid));
      }
      if (settings.containsKey("sta_password")) {
        safeStrcpy(networkSettings.sta_password, settings["sta_password"], sizeof(networkSettings.sta_password));
      }
      if (settings.containsKey("device_comment")) {
        safeStrcpy(networkSettings.device_comment, settings["device_comment"], sizeof(networkSettings.device_comment));
      }
      if (settings.containsKey("ap_mode_enabled")) {
        networkSettings.apModeEnabled = settings["ap_mode_enabled"];
      }
      
      networkSettings.configured = true;
      saveNetworkSettingsToEEPROM();
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleGetNetworkSettings() {
  addCORSHeaders();
  
  DynamicJsonDocument doc(512);
  doc["ssid"] = networkSettings.ssid;
  doc["password"] = networkSettings.password;
  doc["subnet"] = networkSettings.subnet;
  doc["sta_ssid"] = networkSettings.sta_ssid;
  doc["sta_password"] = networkSettings.sta_password;
  doc["device_comment"] = networkSettings.device_comment;
  doc["ap_mode_enabled"] = networkSettings.apModeEnabled;
  doc["configured"] = networkSettings.configured;
  
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// Функция для попытки регистрации на основном шлюзе
void attemptRegistration() {
  if (WiFi.status() == WL_CONNECTED && !registrationAttempted) {
    registrationAttempted = true;
    
    // Получаем IP основного шлюза
    IPAddress gateway = WiFi.gatewayIP();
    
    Serial.print("Attempting registration with gateway: ");
    Serial.println(gateway.toString());
    
    // Отправляем HTTP POST запрос для регистрации
    WiFiClient client;
    if (client.connect(gateway, 80)) {
      Serial.println("Connected to gateway for registration");
      
      // Формируем JSON данные для регистрации
      DynamicJsonDocument doc(512);
      doc["type"] = "device_registration";
      doc["device_name"] = networkSettings.ssid;
      doc["device_ip"] = WiFi.localIP().toString();
      doc["device_mac"] = WiFi.macAddress();
      doc["device_comment"] = networkSettings.device_comment;
      
      String json;
      serializeJson(doc, json);
      
      // Отправляем HTTP POST запрос
      client.println("POST /api/register_device HTTP/1.1");
      client.println("Host: " + gateway.toString());
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.print("Content-Length: ");
      client.println(json.length());
      client.println();
      client.println(json);
      
      Serial.println("Registration data sent via HTTP");
      
      // Ждем ответа
      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 5000) {
        if (client.available()) {
          String line = client.readStringUntil('\n');
          Serial.println(line);
        }
      }
      
      client.stop();
      Serial.println("Registration completed");
    } else {
      Serial.println("Failed to connect to gateway for registration");
    }
  }
}

const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>VR Head Tracker - MPU6050 Sensor Data</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1400px; margin: 0 auto; }
        .header { background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
        .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-bottom: 20px; }
        .stat-card { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center; }
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
        .face { position: absolute; width: 300px; height: 300px; border: 3px solid #34495e; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: bold; color: white; }
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
        
        /* Network devices styles */
        .devices-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 15px; margin-bottom: 20px; }
        .device-card { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .device-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .device-name { font-weight: bold; font-size: 1.1em; }
        .device-ip { color: #666; font-family: monospace; }
        .device-mac { color: #888; font-size: 0.9em; font-family: monospace; }
        .status-online { background: #d4edda; color: #155724; }
        .status-offline { background: #f8d7da; color: #721c24; }
        
        /* Modal styles */
        .modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.5); }
        .modal-content { background: white; margin: 50px auto; padding: 20px; border-radius: 8px; max-width: 500px; }
        .form-group { margin-bottom: 15px; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input, select, textarea { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
        textarea { height: 100px; resize: vertical; }
        .tab-buttons { display: flex; margin-bottom: 20px; }
        .tab-btn { padding: 10px 20px; background: #e9ecef; border: none; cursor: pointer; margin-right: 5px; }
        .tab-btn.active { background: #007bff; color: white; }
        .tab-content { display: none; }
        .tab-content.active { display: block; }
        .wifi-status { padding: 10px; border-radius: 4px; margin-bottom: 15px; }
        .wifi-connected { background: #d4edda; color: #155724; }
        .wifi-disconnected { background: #f8d7da; color: #721c24; }
        .wifi-scanning { background: #fff3cd; color: #856404; }
        .network-list { max-height: 200px; overflow-y: auto; border: 1px solid #ddd; border-radius: 4px; padding: 10px; }
        .network-item { padding: 8px; border-bottom: 1px solid #eee; cursor: pointer; }
        .network-item:hover { background: #f8f9fa; }
        .network-item:last-child { border-bottom: none; }
        .network-ssid { font-weight: bold; }
        .network-info { font-size: 0.8em; color: #666; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>VR Head Tracker - MPU6050 Sensor Data</h1>
            <p>Real-time 3D orientation visualization and network monitoring</p>
            <p><strong>Access Point:</strong> <span id="ap-name">VR_Head_Hom_001</span></p>
            <br/><button class="btn-info" onclick="showSettings()">Network Settings</button>
        </div>
        
        <div class="stats">
            <div class="stat-card">
                <h3>Total Devices</h3>
                <p id="total-devices">0</p>
            </div>
            <div class="stat-card">
                <h3>Online Devices</h3>
                <p id="online-devices">0</p>
            </div>
            <div class="stat-card">
                <h3>ESP IP</h3>
                <p id="esp-ip">0.0.0.0</p>
            </div>
            <div class="stat-card">
                <h3>WiFi Status</h3>
                <p id="wifi-status">-</p>
            </div>
            <div class="stat-card">
                <h3>MPU6050 Status</h3>
                <p id="mpu-status">-</p>
            </div>
        </div>
        
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
            <h3>Angle Control</h3>
            <button class="btn-success" onclick="sendCommand('SET_ZERO')">Set Zero Point</button>
            <button class="btn-danger" onclick="sendCommand('RESET_ANGLES')">Reset All Angles</button>
            <button class="btn-warning" onclick="sendCommand('RESET_ZERO')">Reset Zero Point</button>
            <button class="btn-info" onclick="sendCommand('RESET_OFFSETS')">Reset Zero Offsets</button>
            <div style="margin-top: 15px; padding: 10px; background: white; border-radius: 5px;">
                <div style="font-size: 14px; color: #666;">
                    <strong>Zero Point:</strong> <span id="zeroStatus" style="color: #dc3545; font-weight: bold;">Not Set</span>
                </div>
                <div style="font-size: 12px; color: #888; margin-top: 8px;">
                    <strong>SET_ZERO</strong> - Set current position as reference<br>
                    <strong>RESET_ANGLES</strong> - Reset all angles to zero<br>
                    <strong>RESET_ZERO</strong> - Reset zero point offsets<br>
                    <strong>RESET_OFFSETS</strong> - Clear zero point completely
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
            <h3>Device Controls</h3>
            <button class="btn-primary" onclick="sendCommand('GET_DATA')">Get Sensor Data</button>
            <button class="btn-warning" onclick="sendCommand('RECALIBRATE')">Recalibrate</button>
        </div>

        <div id="devices-container" class="devices-grid">
            <div class="no-devices">No devices found. Click "Scan Network" to discover devices.</div>
        </div>

        <div class="data">
            <h3>API Information</h3>
            <div style="background: #e9ecef; padding: 15px; border-radius: 5px;">
                <p><strong>GET /api/status</strong> - Get device status</p>
                <p><strong>POST /api/setZero</strong> - Set zero point</p>
                <p><strong>POST /api/resetAngles</strong> - Reset all angles</p>
                <p><strong>GET /api/networkSettings</strong> - Get network settings</p>
                <p><strong>POST /api/networkSettings</strong> - Update network settings</p>
            </div>
        </div>
        
        <div class="data">
            <h3>Last Message</h3>
            <div id="lastMessage" style="background: #f8f9fa; padding: 10px; border-radius: 5px; font-family: monospace; min-height: 20px;">No data received</div>
        </div>
    </div>

    <!-- Network Settings Modal -->
    <div id="settings-modal" class="modal">
        <div class="modal-content">
            <h2>Network Settings</h2>
            
            <div class="tab-buttons">
                <button class="tab-btn active" onclick="showTab('ap-settings')">AP Settings</button>
                <button class="tab-btn" onclick="showTab('sta-settings')">WiFi Client</button>
                <button class="tab-btn" onclick="showTab('device-comment')">Device Comment</button>
            </div>
            
            <form id="settings-form">
                <div id="ap-settings" class="tab-content active">
                    <div class="form-group">
                        <label>
                            <input type="checkbox" id="ap-mode-enabled" name="ap_mode_enabled" checked>
                            Enable Access Point Mode
                        </label>
                    </div>
                    <div class="form-group">
                        <label for="ssid">AP SSID:</label>
                        <input type="text" id="ssid" name="ssid" required>
                    </div>
                    <div class="form-group">
                        <label for="password">AP Password:</label>
                        <input type="password" id="password" name="password" required>
                    </div>
                    <div class="form-group">
                        <label for="subnet">Subnet (1-254):</label>
                        <input type="number" id="subnet" name="subnet" min="1" max="254" required>
                    </div>
                </div>
                
                <div id="sta-settings" class="tab-content">
                    <div id="wifi-status-display" class="wifi-status wifi-disconnected">
                        WiFi Status: Not Connected
                    </div>
                    <div class="form-group">
                        <label for="sta-ssid">WiFi SSID:</label>
                        <div style="display: flex;">
                            <input type="text" id="sta-ssid" name="sta_ssid" style="flex: 1;">
                            <button type="button" onclick="showNetworkList()" style="margin-left: 10px;">Select</button>
                        </div>
                    </div>
                    <div class="form-group">
                        <label for="sta-password">WiFi Password:</label>
                        <input type="password" id="sta-password" name="sta_password">
                    </div>
                    <div class="form-group">
                        <button type="button" onclick="scanWiFiNetworks()">Scan Networks</button>
                    </div>
                    <div id="network-list" class="network-list" style="display: none;">
                        <div class="network-item">
                            <div class="network-ssid">Scanning networks...</div>
                        </div>
                    </div>
                </div>
                
                <div id="device-comment" class="tab-content">
                    <div class="form-group">
                        <label for="device-comment-text">Device Comment:</label>
                        <textarea id="device-comment-text" name="device_comment" placeholder="Enter device description or comments..."></textarea>
                    </div>
                    <div style="font-size: 12px; color: #666;">
                        This comment will be sent during device registration to the main gateway.
                    </div>
                </div>
                
                <div style="text-align: right; margin-top: 20px;">
                    <button type="button" onclick="hideSettings()">Cancel</button>
                    <button type="submit">Save Settings</button>
                </div>
            </form>
        </div>
    </div>

    <script>
        let ws = null;
        let statusDiv = document.getElementById('status');
        let lastMessageDiv = document.getElementById('lastMessage');
        let cube = document.getElementById('cube');
        let zeroStatusSpan = document.getElementById('zeroStatus');
        let apNameSpan = document.getElementById('ap-name');
        let devices = [];
        let currentSettings = {};
        
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = protocol + '//' + window.location.hostname + ':81';
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                statusDiv.textContent = 'Connected';
                statusDiv.className = 'status connected';
                document.getElementById('mpu-status').textContent = 'Connected';
                document.getElementById('mpu-status').style.color = '#28a745';
                
                // Request initial data
                sendCommand('GET_DATA');
                sendCommand('SCAN_NETWORK');
                loadCurrentSettings();
            };
            
            ws.onmessage = function(event) {
                console.log('Received:', event.data);
                lastMessageDiv.textContent = event.data;
                
                try {
                    var data = JSON.parse(event.data);
                    handleWebSocketMessage(data);
                    return;
                } catch (e) {
                    // Not JSON, handle as sensor data
                }
                
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
                    const zeroPitchMatch = data.match(/ZERO_PITCH:([-\d.]+)/);
                    const zeroRollMatch = data.match(/ZERO_ROLL:([-\d.]+)/);
                    const zeroYawMatch = data.match(/ZERO_YAW:([-\d.]+)/);
                    
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
                if (event.data === 'ZERO_OFFSETS_RESET') {
                    zeroStatusSpan.textContent = 'Not Set';
                    zeroStatusSpan.style.color = '#dc3545';
                    showNotification('Zero offsets reset', 'info');
                }
                if (event.data === 'All angles reset to zero') {
                    showNotification('All angles reset to zero', 'info');
                }
                if (event.data.startsWith('ZERO_SET:')) {
                    zeroStatusSpan.textContent = 'Set';
                    zeroStatusSpan.style.color = '#28a745';
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                statusDiv.textContent = 'Disconnected';
                statusDiv.className = 'status disconnected';
                document.getElementById('mpu-status').textContent = 'Disconnected';
                document.getElementById('mpu-status').style.color = '#dc3545';
                
                setTimeout(connectWebSocket, 2000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function handleWebSocketMessage(data) {
            switch(data.type) {
                case 'devices_list':
                    updateDevicesList(data);
                    break;
                case 'device_update':
                    updateDevice(data.device);
                    break;
                case 'wifi_networks':
                    updateWifiNetworks(data.networks);
                    break;
                case 'current_settings':
                    currentSettings = data.settings;
                    updateSettingsForm();
                    break;
                case 'settings_updated':
                    showNotification('Settings updated successfully', 'success');
                    break;
            }
        }
        
        function updateDevicesList(data) {
            devices = data.devices || [];
            
            // Update statistics
            document.getElementById('total-devices').textContent = data.totalDevices || 0;
            document.getElementById('online-devices').textContent = data.onlineDevices || 0;
            document.getElementById('esp-ip').textContent = data.espIp || '0.0.0.0';
            
            var wifiStatus = document.getElementById('wifi-status');
            if (data.wifiStatus === 'connected') {
                wifiStatus.textContent = 'Connected';
                wifiStatus.style.color = '#28a745';
            } else {
                wifiStatus.textContent = 'Disconnected';
                wifiStatus.style.color = '#dc3545';
            }
            
            // Update AP name
            if (currentSettings.ssid) {
                apNameSpan.textContent = currentSettings.ssid;
            }
            
            // Update devices grid
            var container = document.getElementById('devices-container');
            
            if (devices.length === 0) {
                container.innerHTML = '<div class="no-devices">No devices found. Click "Scan Network" to discover devices.</div>';
                return;
            }
            
            container.innerHTML = '';
            
            devices.forEach(function(device) {
                var deviceCard = createDeviceCard(device);
                container.appendChild(deviceCard);
            });
        }
        
        function createDeviceCard(device) {
            var card = document.createElement('div');
            card.className = 'device-card';
            
            var statusClass = device.connected ? 'status-online' : 'status-offline';
            var statusText = device.connected ? 'Online' : 'Offline';
            
            card.innerHTML = '<div class="device-header">' +
                '<div class="device-name">' + escapeHtml(device.hostname) + '</div>' +
                '</div>' +
                '<div class="device-ip">' + escapeHtml(device.ip) + '</div>' +
                '<div class="device-mac">' + escapeHtml(device.mac) + '</div>';
            
            return card;
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
        
        function scanNetwork() {
            sendCommand('SCAN_NETWORK');
        }
        
        function scanWiFiNetworks() {
            sendCommand('SCAN_WIFI');
            document.getElementById('wifi-status-display').className = 'wifi-status wifi-scanning';
            document.getElementById('wifi-status-display').textContent = 'Scanning WiFi networks...';
        }
        
        function updateWifiNetworks(networks) {
            var networkList = document.getElementById('network-list');
            networkList.innerHTML = '';
            
            if (networks && networks.length > 0) {
                networks.forEach(function(network) {
                    var networkItem = document.createElement('div');
                    networkItem.className = 'network-item';
                    networkItem.innerHTML = '<div class="network-ssid">' + escapeHtml(network.ssid) + '</div>' +
                        '<div class="network-info">RSSI: ' + network.rssi + ' dBm, ' + network.encryption + '</div>';
                    networkItem.onclick = function() {
                        document.getElementById('sta-ssid').value = network.ssid;
                        networkList.style.display = 'none';
                    };
                    networkList.appendChild(networkItem);
                });
                
                document.getElementById('wifi-status-display').className = 'wifi-status wifi-disconnected';
                document.getElementById('wifi-status-display').textContent = 'Found ' + networks.length + ' networks';
            } else {
                document.getElementById('wifi-status-display').className = 'wifi-status wifi-disconnected';
                document.getElementById('wifi-status-display').textContent = 'No networks found';
            }
        }
        
        function loadCurrentSettings() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({type: 'get_settings'}));
            }
        }
        
        function updateSettingsForm() {
            document.getElementById('ssid').value = currentSettings.ssid || '';
            document.getElementById('password').value = currentSettings.password || '';
            document.getElementById('subnet').value = currentSettings.subnet || '50';
            document.getElementById('sta-ssid').value = currentSettings.sta_ssid || '';
            document.getElementById('sta-password').value = currentSettings.sta_password || '';
            document.getElementById('device-comment-text').value = currentSettings.device_comment || '';
            document.getElementById('ap-mode-enabled').checked = currentSettings.ap_mode_enabled !== false;
            
            // Update AP name display
            if (currentSettings.ssid) {
                apNameSpan.textContent = currentSettings.ssid;
            }
        }
        
        function showSettings() {
            loadCurrentSettings();
            document.getElementById('settings-modal').style.display = 'block';
        }
        
        function hideSettings() {
            document.getElementById('settings-modal').style.display = 'none';
        }
        
        function showTab(tabId) {
            // Hide all tabs
            var tabs = document.querySelectorAll('.tab-content');
            tabs.forEach(function(tab) {
                tab.classList.remove('active');
            });
            var buttons = document.querySelectorAll('.tab-btn');
            buttons.forEach(function(btn) {
                btn.classList.remove('active');
            });
            
            // Show selected tab
            document.getElementById(tabId).classList.add('active');
            event.target.classList.add('active');
        }
        
        function showNetworkList() {
            var networkList = document.getElementById('network-list');
            networkList.style.display = networkList.style.display === 'none' ? 'block' : 'none';
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
        
        function escapeHtml(unsafe) {
            return unsafe
                .replace(/&/g, "&amp;")
                .replace(/</g, "&lt;")
                .replace(/>/g, "&gt;")
                .replace(/"/g, "&quot;")
                .replace(/'/g, "&#039;");
        }

        // Form submissions
        document.getElementById('settings-form').addEventListener('submit', function(e) {
            e.preventDefault();
            var formData = new FormData(this);
            var settings = {
                ssid: formData.get('ssid'),
                password: formData.get('password'),
                subnet: formData.get('subnet'),
                sta_ssid: formData.get('sta_ssid'),
                sta_password: formData.get('sta_password'),
                device_comment: formData.get('device_comment'),
                ap_mode_enabled: document.getElementById('ap-mode-enabled').checked
            };
            
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'update_network_settings',
                    settings: settings
                }));
            }
            
            hideSettings();
        });

        // Initialize when page loads
        window.addEventListener('load', function() {
            connectWebSocket();
        });
        
        // Close modals when clicking outside
        window.onclick = function(event) {
            var modals = document.getElementsByClassName('modal');
            for (var i = 0; i < modals.length; i++) {
                if (event.target == modals[i]) {
                    modals[i].style.display = 'none';
                }
            }
        };
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", MAIN_page);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting VR Head Tracker...");

  // Инициализация EEPROM
  EEPROM.begin(2048);
  
  // Загрузка настроек
  loadNetworkSettingsFromEEPROM();
  
  // Настройка точки доступа
  int subnet = atoi(networkSettings.subnet);
  if (subnet < 1 || subnet > 254) {
    subnet = 50;
  }
  
  if (networkSettings.apModeEnabled) {
    IPAddress local_ip(192, 168, subnet, 1);
    IPAddress gateway(192, 168, subnet, 1);
    IPAddress subnet_mask(255, 255, 255, 0);
    
    WiFi.softAPConfig(local_ip, gateway, subnet_mask);
    
    if (WiFi.softAP(networkSettings.ssid, networkSettings.password)) {
      Serial.println("WiFi AP started successfully");
      Serial.printf("SSID: %s\n", networkSettings.ssid);
      Serial.printf("IP address: %s\n", WiFi.softAPIP().toString().c_str());
    } else {
      Serial.println("Failed to start WiFi AP");
    }
  }
  
  // Подключение к WiFi как клиент (если настроено)
  if (strlen(networkSettings.sta_ssid) > 0) {
    Serial.print("Connecting to WiFi ");
    Serial.println(networkSettings.sta_ssid);
    
    WiFi.begin(networkSettings.sta_ssid, networkSettings.sta_password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(1000);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
      
      // Пытаемся зарегистрироваться на основном шлюзе
      attemptRegistration();
    } else {
      Serial.println("\nFailed to connect to WiFi");
    }
  }
  
  // Инициализация MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    delay(1000);
    ESP.restart();
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  
  calibrateSensor();
  
  // Инициализация DHCP сервера
  if (networkSettings.apModeEnabled) {
    initDHCPServer();
  }
  
  // Настройка веб-сервера
  server.on("/", handleRoot);
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/setZero", HTTP_POST, handleSetZero);
  server.on("/api/resetAngles", HTTP_POST, handleResetAngles);
  server.on("/api/networkSettings", HTTP_GET, handleGetNetworkSettings);
  server.on("/api/networkSettings", HTTP_POST, handleNetworkSettings);
  
  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/setZero", HTTP_OPTIONS, handleOptions);
  server.on("/api/resetAngles", HTTP_OPTIONS, handleOptions);
  server.on("/api/networkSettings", HTTP_OPTIONS, handleOptions);
  
  server.enableCORS(true);
  server.begin();
  
  // Запуск WebSocket сервера
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("HTTP server started on port 80");
  Serial.println("WebSocket server started on port 81");
  
  // Первоначальное сканирование сети
  scanNetwork();
  
  Serial.println("Setup completed");
  Serial.println("VR Head Tracker available at: " + WiFi.softAPIP().toString());
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  if (networkSettings.apModeEnabled) {
    handleDHCP();
  }
  
  if (!calibrated) return;
  
  // Чтение данных с MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (lastTime == 0) deltaTime = 0.01;
  lastTime = currentTime;
  
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;
  
  // Комплементарный фильтр для стабилизации углов
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  // Обновление углов гироскопом
  pitch += gyroX * deltaTime * 180.0 / PI;
  roll += gyroY * deltaTime * 180.0 / PI;
  yaw += gyroZ * deltaTime * 180.0 / PI;
  
  // Комплементарный фильтр для стабилизации
  float alpha = 0.96;
  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll = alpha * roll + (1.0 - alpha) * accelRoll;
  
  // Отправка данных через WebSocket
  if (clientConnected && (currentTime - lastDataSend >= SEND_INTERVAL)) {
    if (dataChanged() || lastDataSend == 0) {
      sendSensorData();
      lastDataSend = currentTime;
    }
  }
  
  // Периодическое сканирование сети
  if (currentTime - lastScanTime > SCAN_INTERVAL) {
    scanNetwork();
  }
  
  // Периодическое сохранение в EEPROM
  if (eepromDirty && (currentTime - lastEEPROMSave > EEPROM_SAVE_INTERVAL)) {
    saveNetworkSettingsToEEPROM();
    eepromDirty = false;
    lastEEPROMSave = currentTime;
  }
  
  delay(10);
}
