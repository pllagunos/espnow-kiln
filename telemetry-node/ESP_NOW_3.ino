// ESP NOW USES ESP32 2.0.11 in board manager

/* Mac Addresses */
// ESP32 #1: 08:3A:F2:A9:7E:DC <- Main controller (0C:B8:15:C3:80:34)
// ESP32 #2: 08:3A:F2:6C:CB:9C <- Publishes EAST DPTs and opening chimney
// ESP32 #3: 40:91:51:BF:CF:FC <--- THIS ONE, publishing WEST DPTs, receiving EVERYTHING

// NEG = A0, NEA = A1, SEG = A2, SEA = A3

#include <esp_now.h>
#include <esp_wifi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <WiFi.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#define DO1 16
#define DO2 17
#define DO3 13 

// MAC Addresses
uint8_t ESP32_1[] = {0x08, 0x3A, 0xF2, 0xA9, 0x7E, 0xDC};
uint8_t ESP32_2[] = {0x08, 0x3A, 0xF2, 0x6C, 0xCB, 0x9C}; 
esp_now_peer_info_t peerInfo;

// WIFI SSID and PASSWORD
String ssid = "your-ssid-here";
String password = "your-password-here";

// Refresh Cycles
const int displayCycle = 1500;                  // OLED display refresh cycle(ms)
const int dptCycle = 2500;                      // DPT sensors reading cycle (ms)
const int influxCycle = 10000;                   // Influx DB publishing cycle (ms)
const int EspNowTimeOut = 10000;                // (ms) before espnow timeouts

// Pin number connections
const int A[] = {36, 39, 35, 33};               // GPIOs VP,VN,35,33
const int R[] = {100, 99, 99, 100};             // Resistors used (ohms)
const int LED = DO1;                             // Pin number for LED

// Other variables
const int vmin = 350;                           // minimum voltage range for ADC
const int amin = 285;                           // experimental ADC adcValue for vmin
const int vmax = 2150;                          // maximum voltage range for ADC
const int amax = 2504;                          // experimental ADC adcValue for vmax
const int iterations = 20;                      // iterations for ADC
const int inputs = 4;                           // Number of ADC inputs
int adcValue[inputs];
float voltage[inputs];
float current[inputs];
float pressure[inputs];
float incomingTemp;
unsigned long influxTime = 0;
unsigned long displayTime = 0;
unsigned long dptTime = 0;
unsigned long ESP32_2_msgTime = 0;
unsigned long ESP32_1_msgTime = 0;
volatile bool dataSent = false;
volatile bool ESP32_1_isOK = false;
volatile bool ESP32_2_isOK = false;
bool Influx_isOK = false;
bool connected = false;
bool prevConnected = false;
int fails = 0;
int8_t WiFiQuality;

/* Sender structure: must match the receiver structure */
typedef struct DPT_message {
  //char id[4]; // east or west
  float p[inputs]; // 2 gas pressures, 2 air pressures
} DPT_message;
/* Receiver structure: must match the sender structure */
typedef struct kiln_message {
  int temp; // pidInput
  int fuegoBajo; // FB
  int fuegoAlto; // FA
  int fuegoSuperAlto; // FSA
  int SetPoint; // pidSetPoint
  int Output;  // pidOutput 0%-100%
} kiln_message;

// Create messages
volatile DPT_message DPT_W; // DPT_W values to send
volatile DPT_message DPT_E; // DPT_E values to receive
volatile kiln_message Kiln; // Furnace values to receive

/* Cloud DB settings */
#define INFLUXDB_URL "influxdb-url"
#define INFLUXDB_TOKEN "token-here"
#define INFLUXDB_ORG "account-name"
#define INFLUXDB_BUCKET "bucket-name"
#define TZ_INFO "CST+6CDT,M4.1.0/2,M10.5.0/2" // your timezone

InfluxDBClient* client = nullptr;
Point sensor("HORNO1");
SSD1306Wire  display(0x3c, SDA, SCL); // SDA(G21) and SCL(G22) default populate 

void setup() {
  Serial.begin(115200);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  //memcpy(DPT_W.id, "west", strlen("west") + 1);
  for (int i = 0; i < inputs; i++) {
    pinMode(A[i], INPUT);
  }
  pinMode(LED, OUTPUT);
  
  display.init();
  display.flipScreenVertically();

  // Initialize WiFi
  initWiFi();

  // Now that WiFi is connected, initialize ESP-NOW
  initEspNow();

}

void loop() {
  // Refresh OLED display
  if (millis() - displayTime >= displayCycle) {
    displayTime = millis();
    display.clear();
    drawState();
    display.display();
  }

  // Update DPT readings and send data through ESP-NOW
  if (millis() - dptTime >= dptCycle) {
    getDPTreadings();    
    // Assign pressures to structure
    for (int i = 0; i < inputs; i++) DPT_W.p[i] = pressure[i];
    
    // Print pressures
    // Serial.printf("\n ESP #3. Presiones medidas: \n NWG: %f in. WC \n NWA: %f in. WC \n SWG: %f in. WC \n SWA: %f in. WC\n", DPT_W.p[0], DPT_W.p[1], DPT_W.p[2], DPT_W.p[3]);    
    
    // Send pressure via ESP-NOW
    esp_err_t result = esp_now_send(ESP32_1, (uint8_t *) &DPT_W, sizeof(DPT_W));
    if (result != ESP_OK) {
      Serial.print("Error sending the data: 0x");
      Serial.println((uint32_t)result, HEX); // Cast result to ensure proper printing in HEX
      Serial.println(esp_err_to_name(result)); // Print the name of the error
      dataSent = false; // immediate failure to send
    }

    dptTime = millis();
  }

  // Upload data to Influx and send temperature
  if ( millis() - influxTime >= influxCycle) {
    // check wifi
    prevConnected = connected;
    if (WiFi.status() != WL_CONNECTED) {
      initWiFi(); // reconnect if no connection
      connected = false;
      Influx_isOK = false;
      return;
    }
    else connected = true;
    
    // if needed (re)initialize influx client
    if (!prevConnected && connected || fails > 4) {
      Serial.println("reinitializing influx");

      fails = 0;
      delay(2500);

      // Reinitialize InfluxDBClient
      if (client != nullptr) {
        delete client; // Ensure previous instance is deleted
      }
      client = new InfluxDBClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
      timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
      // client->setInsecure();

      if (client->validateConnection()) {
        Serial.print("Connected to InfluxDB: ");
        Serial.println(client->getServerUrl());
      } else {
        Serial.print("InfluxDB connection failed: ");
        Serial.println(client->getLastErrorMessage());
        Influx_isOK = false;
        fails = 5; // force time sync and validation again
        return; 
      }
    }

    // if we get here, try publishing!

    DPT_message dpt_w, dpt_e; // DPT local values
    kiln_message kiln; // kiln local values

    // Disable interrupts to make a thread-safe copy of the data
    noInterrupts();
    copyDPTMessage(DPT_W, dpt_w);
    copyDPTMessage(DPT_E, dpt_e);
    copyKilnMessage(Kiln, kiln);
    interrupts(); // Re-enable interrupts

    sensor.clearFields();
    
    // Report RSSI of currently connected network
    // sensor.addField("rssi", WiFi.RSSI());

    sensor.addField("Pressure NEG", dpt_e.p[0]); sensor.addField("Pressure NWG", dpt_w.p[0]);
    sensor.addField("Pressure NEA", dpt_e.p[1]); sensor.addField("Pressure NWA", dpt_w.p[1]);
    sensor.addField("Pressure SEG", dpt_e.p[2]); sensor.addField("Pressure SWG", dpt_w.p[2]);
    sensor.addField("Pressure SEA", dpt_e.p[3]); sensor.addField("Pressure SWA", dpt_w.p[3]);
    sensor.addField("Kiln temperature", kiln.temp);
    sensor.addField("Fuego Bajo", kiln.fuegoBajo);
    sensor.addField("Fuego Alto", kiln.fuegoAlto);
    sensor.addField("Fuego Super Alto", kiln.fuegoSuperAlto);
    sensor.addField("PID SetPoint", kiln.SetPoint);
    sensor.addField("PID Output", kiln.Output);
    
    sensor.addField("WiFi %", WiFiQuality);

    // write data
    Influx_isOK = (client->writePoint(sensor));
    if (!Influx_isOK) {
      Serial.print("InfluxDB write failed:");
      Serial.println(client->getLastErrorMessage());
      fails += 1;
    }
    else {
      Serial.print("Succesfully wrote this: \n");
      Serial.println(client->pointToLineProtocol(sensor));
    }

    influxTime = millis();
  }

  // Check if ESP messages have been updated
  if (millis() - ESP32_2_msgTime >= EspNowTimeOut)  ESP32_2_isOK = false;
  else                                              ESP32_2_isOK = true;   
  if (millis() - ESP32_1_msgTime >= EspNowTimeOut)  ESP32_1_isOK = false;
  else                                              ESP32_1_isOK = true;
  if (ESP32_1_isOK && ESP32_2_isOK && Influx_isOK)  digitalWrite(LED, LOW);
  else                                              digitalWrite(LED, HIGH); // error means red led is ON

}

//******************************************************************************************************************************
//  DRAWSTATE: DRAWS SYSTEM STATE ON OLED DISPLAY
//******************************************************************************************************************************
void drawState() {
  display.setFont(ArialMT_Plain_10);
  display.setColor(WHITE);
  drawWifiQuality(); 
  int x = 0;
  int y = 15;
  //ifs
  if (Influx_isOK)      display.drawString(x, y, "InfluxDB OK");
  else                  display.drawString(x, y, "InfluxDB NO SIGNAL");
  if ( ESP32_1_isOK )   display.drawString(x, y + 10, "ESP #1 OK");
  else                  display.drawString(x, y + 10, "ESP #1 NO SIGNAL");
  if ( ESP32_2_isOK )   display.drawString(x, y + 20, "ESP #2 OK");
  else                  display.drawString(x, y + 20, "ESP #2 NO SIGNAL");
}

//******************************************************************************************************************************
//  GETDPTREADINGS: SAVES PRESSURE,CURRENT,VOLTAGE AND ADVALUE OF DPT SENSORS
//******************************************************************************************************************************
void getDPTreadings() {
  for (int i = 0; i < inputs; i++) {
    for (int j = 0; j < iterations; j++) {
      adcValue[i] += analogRead(A[i]);
      delay(5);
    }
    adcValue[i] = adcValue[i] / iterations;                        // average ADC adcValue
    voltage[i] = mapfloat(adcValue[i], amin, amax, vmin, vmax);    // empirical ADC adcValue to voltage adcValue
    current[i] = mapfloat(voltage[i], 4 * R[i], 20 * R[i], 4, 20); // voltage to mA (ohm's law)
    pressure[i] = mapfloat(current[i], 4, 20, 0, 25);              // mA to pressure ( 0 - 25 in WC )
    pressure[i] = round(pressure[i] * 4) / 4.0;                    // round to nearest 1/4"
  }
}

//******************************************************************************************************************************
//  ONDATARECV: CALLBACK FUNCTION THAT WILL BE EXECTUED WHEN DATA IS RECEIVED
//******************************************************************************************************************************
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // Get mac address from sender
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // memcpy data to appropiate struct
  if (memcmp(mac_addr, ESP32_2, sizeof(mac_addr)) == 0) { 
    //Serial.printf("\n Packet received from: %s,   AKA ESP32 #2\n", macStr);
    copyToVolatile(&DPT_E, incomingData, sizeof(DPT_E));
    // memcpy(&DPT_E, incomingData, sizeof(DPT_E));
    //Serial.printf("\n Message length: %u bytes\n", len);  
    //Serial.printf("\n Presiones recibidas: \n NEG: %f in. WC \n NEA: %f in. WC \n SEG: %f in. WC \n SEA: %f in. WC \n",
    //DPT_E.p[0], DPT_E.p[1], DPT_E.p[2], DPT_E.p[3]);
    ESP32_2_msgTime = millis();
  }  
  if (memcmp(mac_addr, ESP32_1, sizeof(mac_addr)) == 0) { 
    //Serial.printf("\n Packed received from: %s,   AKA ESP32 #1\n", macStr);
    copyToVolatile(&Kiln, incomingData, sizeof(Kiln));
    // memcpy(&Kiln, incomingData, sizeof(Kiln));
    //Serial.printf(" Message length: %u bytes\n", len);  
    //Serial.printf("\n Temperatura: %d °C \n", Kiln.temp);
    if (dataSent) ESP32_1_msgTime = millis();
  }  
  
}

//******************************************************************************************************************************
//  ONDATASENT: CALLBACK FUNCTION THAT WILL BE EXECUTED WHEN DATA IS SENT
//******************************************************************************************************************************
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Get mac address from receiver
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);         
  if (memcmp(mac_addr, ESP32_1, sizeof(mac_addr)) == 0) {
    Serial.printf("\n Packet sent to ESP32 #1\t status:\t %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail" );
  }
  if (memcmp(mac_addr, ESP32_2, sizeof(mac_addr)) == 0) {
    Serial.printf("\n Packet sent to ESP32 #2\t status:\t %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail" );
  }
  // Update dataSent based on the callback status
  dataSent = (status == ESP_NOW_SEND_SUCCESS);
}
//******************************************************************************************************************************
//  INITESPNOW: INITIALIZE ESP NOW
//******************************************************************************************************************************
void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW failed to initialize");
    digitalWrite(LED, HIGH); // show fail
    delay(3000);
    esp_restart();
  }

  peerInfo.ifidx   = WIFI_IF_STA; 
  peerInfo.encrypt = false;

  // Attempt to add first peer (ESP32 #1)
  memcpy(peerInfo.peer_addr, ESP32_1, 6);
  for (int attempts = 0; attempts < 3; attempts++) {
    // digitalWrite(LED, HIGH); // Turn on LED to indicate an attempt is being made
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("ESP #1 pairing success");
      ESP32_1_isOK = true;
      digitalWrite(LED, LOW); // Turn off LED on success
      break; // Exit loop on success
    } else {
      Serial.println("Attempt to pair ESP #1 failed, retrying...");
      digitalWrite(LED, LOW); // Blink LED to indicate retry
      delay(500); // Wait a bit before retrying, LED is off during this wait
    }
  }
  // Attempt to add second peer (ESP32 #2)
  memcpy(peerInfo.peer_addr, ESP32_2, 6);
  for (int attempts = 0; attempts < 3; attempts++) {
    digitalWrite(LED, HIGH); // Turn on LED to indicate an attempt is being made
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("ESP #2 pairing success");
      ESP32_2_isOK = true;
      digitalWrite(LED, LOW); // Turn off LED on success
      break; // Exit loop on success
    } else {
      Serial.println("Attempt to pair ESP #2 failed, retrying...");
      digitalWrite(LED, LOW); // Blink LED to indicate retry
      delay(500); // Wait a bit before retrying, LED is off during this wait
    }
  }
  
  // ***** Register Send CB and Recv CB *****
  esp_now_register_send_cb(OnDataSent);  
  esp_now_register_recv_cb(OnDataRecv);
}

//******************************************************************************************************************************
// copyToVolatile: Function to safely copy data to a volatile destination
//******************************************************************************************************************************
void copyToVolatile(volatile void* dest, const void* src, size_t n) {
  // Correctly cast away 'volatile' before casting the type
  uint8_t* d = reinterpret_cast<uint8_t*>(const_cast<void*>(dest));
  const uint8_t* s = reinterpret_cast<const uint8_t*>(src);
  while (n--) {
      *d++ = *s++;
  }
}

//******************************************************************************************************************************
//  copyDPTMessage: parses the volatile DPT message to a local copy
//******************************************************************************************************************************
void copyDPTMessage(const volatile DPT_message& src, DPT_message& dest) {
  for (int i = 0; i < inputs; ++i) {
    dest.p[i] = src.p[i];
  }
}

//******************************************************************************************************************************
//  copyKilnMessage: parses the volatile kiln message to a local copy
//******************************************************************************************************************************
void copyKilnMessage(const volatile kiln_message& src, kiln_message& dest) {
  dest.temp = src.temp;
  dest.fuegoBajo = src.fuegoBajo;
  dest.fuegoAlto = src.fuegoAlto;
  dest.fuegoSuperAlto = src.fuegoSuperAlto;
  dest.SetPoint = src.SetPoint;
  dest.Output = src.Output;
}

//******************************************************************************************************************************
//  INITWIFI: INITIALIZE WIFI
//******************************************************************************************************************************
void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  // disp connecting
  Serial.printf("Connecting to %s .", ssid);
  display.setColor(BLACK);
  display.fillRect(60,0,128-60,14);
  display.setColor(WHITE);
  display.drawString(60,0,"Connecting...");
  display.display();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("...");
  }
  Serial.print("\nConnected to:\t");
  Serial.println(WiFi.SSID());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
}

//******************************************************************************************************************************
//  DRAWWIFIQUALITY: DRAW WIFI BARS AND PERCENTAGE
//******************************************************************************************************************************
void drawWifiQuality() {  
  if ( WiFi.status() == WL_CONNECTED ) {  
    WiFiQuality = getWifiQuality();
    display.drawString(108, 0, String(WiFiQuality)+"%");
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 2 * (i + 1); j++) {
        if (WiFiQuality > i * 25){ // || j == 0) {
          display.setPixel(98 + 2 * i, 8-j);
        }
      }
    }
  }
  else{
    display.drawString(80, 0, "OFFLINE");
  }
}

//******************************************************************************************************************************
//  GETWIFIQUALITY: GETS RSSI AND CONVERTS IT FROM DBM TO %
//******************************************************************************************************************************
int8_t getWifiQuality() {
  int32_t dbm = WiFi.RSSI();
  if (dbm <= -100) {
    return 0;
  } else if (dbm >= -50) {
    return 100;
  } else {
    return 2 * (dbm + 100);
  }
}

//******************************************************************************************************************************
//  MAPFLOAT: MAPS LINEAR TRANSFORMATION OF FLOAT VALUES
//******************************************************************************************************************************
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
