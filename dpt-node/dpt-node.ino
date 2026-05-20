// ESP NOW USES ESP32 2.0.11 in board manager

/* Mac Addresses */
// ESP32 #1: 08:3A:F2:A9:7E:DC <- Main controller (0C:B8:15:C3:80:34)
// ESP32 #2: 08:3A:F2:6C:CB:9C <--- THIS ONE, publishing EAST DPTs, opening chimney
// ESP32 #3: 40:91:51:BF:CF:FC <- Publishes WEST DPTs, receives everything

// NEG = A0, NEA = A1, SEG = A2, SEA = A3

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define DO1 16
#define DO2 17
#define DO3 13 

// MAC Addresses
uint8_t ESP32_1[] = {0x08, 0x3A, 0xF2, 0xA9, 0x7E, 0xDC};
uint8_t ESP32_3[] = {0x40, 0x91, 0x51, 0xBF, 0xCF, 0xFC};
esp_now_peer_info_t peerInfo;

// WIFI SSID
constexpr char WIFI_SSID[] = "your-ssid-here";

// refresh cycles
const int dptCycle = 2500;                      // sensor reading cycle
const int EspNowTimeOut = 10000;                // (ms) before espnow timeouts

// Pin number connections
const int chimneyDO = DO1;                      // Chimney output pin
const int A[] = {36, 39, 35, 33};               // GPIOs VP,VN,35,33 ADC1
const int R[] = {99, 100, 99, 100};             // (ohms)

// Other variables
const int vmin = 350;                           // minimum voltage range for ADC (mV)
const int amin = 285;                           // experimental ADC adcValue for vmin
const int vmax = 2150;                          // maximum voltage range for ADC (mV)
const int amax = 2504;                          // experimental ADC adcValue for vmax
const int iterations = 20;                      // iterations for ADC
const int inputs = sizeof(A) / sizeof(int);
int adcValue[inputs];
float voltage[inputs];
float current[inputs];
float pressure[inputs];
bool incomingOpen;
unsigned long DPTmillis = 0;
unsigned long ESP32_1_msgTime = 0;
unsigned long ESP32_3_msgTime = 0;
volatile bool dataSent = false;
volatile bool ESP32_1_isOK = false;
volatile bool ESP32_3_isOK = false;
int fails = 0;

/* Sender structure: must match the receiver structure */
typedef struct DPT_message {
  //char id[4]; // east or west
  float p[inputs]; // 2 gas pressures, 2 air pressures
} DPT_message;
typedef struct chimney_message {
  bool OPEN;
} chimney_message;

// Create messages
volatile DPT_message DPT_E;
volatile chimney_message chimney;

void setup() {
  Serial.begin(115200);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  delay(1000);
  initWiFi();
  initEspNow();

  //memcpy(DPT_E.id, "East", strlen("East") + 1);
  for (int i = 0; i < inputs; i++) {
    pinMode(A[i], INPUT);
  }

  pinMode(chimneyDO, OUTPUT);
  digitalWrite(chimneyDO, HIGH); // standby is on HIGH
}

void loop() {
  if (millis() - DPTmillis >= dptCycle) {
    getDPTreadings();
    // Assign pressures to structure
    for (int i = 0; i < inputs; i++) DPT_E.p[i] = pressure[i];
    
    // Print pressures
    //Serial.printf("\n ESP #2. Presiones medidas: \n NEG: %f in. WC \n NEA: %f in. WC \n SEG: %f in. WC \n SEA: %f in. WC\n", DPT_E.p[0], DPT_E.p[1], DPT_E.p[2], DPT_E.p[3]);
    //Serial.printf("\n ESP #2. Presiones medidas: \n NEG: %f in. WC \n NEA: %f in. WC \n SEG: %f in. WC \n SEA: %f in. WC\n", pressure[0], pressure[1], pressure[2], pressure[3]);
    
    // Send pressures to ESP#1 and ESP#3 via ESP-NOW
    esp_err_t result = esp_now_send(0, (uint8_t *) &DPT_E, sizeof(DPT_E));
    if (result != ESP_OK) {
      Serial.print("Error sending the data: 0x");
      Serial.println((uint32_t)result, HEX); // Cast result to ensure proper printing in HEX
      Serial.println(esp_err_to_name(result)); // Print the name of the error
      dataSent = false; // immediate failure to send
    }

    DPTmillis = millis();
  }

  // Check that messages have been sent succesfully in time
  if (millis() - ESP32_1_msgTime >= EspNowTimeOut)  ESP32_1_isOK = false;
  else                                              ESP32_1_isOK = true; 
  if (millis() - ESP32_3_msgTime >= EspNowTimeOut)  ESP32_3_isOK = false;
  else                                              ESP32_3_isOK = true; 

  // check if esp-now network changed channel
  if (!ESP32_1_isOK || !ESP32_3_isOK) {
    fails += 1;
  } 
  if (fails > 4) {
    Serial.println("reinitializing wifi chanel");  
    initWiFi();
    fails = 0;
  }
}

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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // memcpy data to relevant variable
  copyToVolatile(&chimney, incomingData, sizeof(chimney));
  if (chimney.OPEN == true) {
    Serial.println("open chimney");
    digitalWrite(chimneyDO, LOW);
    delay(100);
    // then send 2 quick pulses
    digitalWrite(chimneyDO, HIGH);
    delay(50);
    digitalWrite(chimneyDO, LOW);
    delay(50);
    digitalWrite(chimneyDO, HIGH);
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
  // Update dataSent based on the callback status
  dataSent = (status == ESP_NOW_SEND_SUCCESS);

  if (dataSent) {
    if (memcmp(mac_addr, ESP32_1, sizeof(mac_addr)) == 0) {
      Serial.printf("\n Packet sent to ESP32 #1\t status:\t %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail" );
      ESP32_1_msgTime = millis();
    }
    if (memcmp(mac_addr, ESP32_3, sizeof(mac_addr)) == 0) {
      Serial.printf("\n Packet sent to ESP32 #3\t status:\t %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail" );
      ESP32_3_msgTime = millis();
    }       
  }

}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW failed to initialize");
    while (1);
  }
  peerInfo.ifidx   = WIFI_IF_STA;
  peerInfo.encrypt = false;

  // Add first peer (ESP32 #1)
  memcpy(peerInfo.peer_addr, ESP32_1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP #1 pairing failure");
    while (1);
  }
  else Serial.println("ESP #1 pairing success");
  
  // Add second peer (ESP32 #3)
  memcpy(peerInfo.peer_addr, ESP32_3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP #3 pairing failure");
    while (1);
  }  
  else Serial.println("ESP #3 pairing success");
  
  // ***** Register Send CB and Recv CB *****
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

// copyToVolatile: Function to safely copy data to a volatile destination
void copyToVolatile(volatile void* dest, const void* src, size_t n) {
  // Correctly cast away 'volatile' before casting the type
  uint8_t* d = reinterpret_cast<uint8_t*>(const_cast<void*>(dest));
  const uint8_t* s = reinterpret_cast<const uint8_t*>(src);
  while (n--) {
      *d++ = *s++;
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  // WiFi.printDiag(Serial); Uncomment to verify channel change before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // WiFi.printDiag(Serial); Uncomment to verify channel change after
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
