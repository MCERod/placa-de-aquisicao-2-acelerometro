#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX: GPIO 4, TX: GPIO 5

#define TAM 7 // Limit of data to be sent (array size 8)

// ADXL345 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
unsigned long timestamp = 0;
int distance;
unsigned char data[4]; // Array to store incoming serial data

// Structure for storing sensor data
typedef struct struct_message {
  float ax;
  float ay;
  float az;
  int distance;
  int tempo;
} struct_message;

// Array of struct_message to store sensor data
struct_message myData[TAM];

// Flag to indicate when to send data
bool sendDataFlag = false;

// Flag to indicate UART read trigger
volatile bool readUARTFlag = false;

// Requester's MAC address
uint8_t requesterMAC[6];

// Hardware timer handle
hw_timer_t *timer = NULL;

// Timer interval (in microseconds)
#define TIMER_INTERVAL_US 150000  // 150 ms interval

// Function prototypes
void sendSensorData(void);
void printMAC(const uint8_t *mac, const char *label);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void IRAM_ATTR onTimer();  // Interrupt Service Routine (ISR) for the timer
void displayDataRate(void);
void displayRange(void);

void printMAC(const uint8_t *mac, const char *label) {
  Serial.print(label);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.println("Received signal to send data");
  memcpy(requesterMAC, mac_addr, 6); // Store the requester's MAC address
  printMAC(requesterMAC, "Responder to: ");
  sendDataFlag = true;
  timestamp = millis();
}

void IRAM_ATTR onTimer() {
  readUARTFlag = true; // Set the flag to read data from the UART
}

void setupTimer() {
  timer = timerBegin(0, 80, true); // Timer 0, prescaler of 80 (1 tick = 1 µs)
  timerAttachInterrupt(timer, &onTimer, true); // Attach ISR to the timer
  timerAlarmWrite(timer, TIMER_INTERVAL_US, true); // Set alarm interval to 150 ms
  timerAlarmEnable(timer); // Enable the timer alarm
}

void displayDataRate(void) {
  Serial.print("Data Rate:    ");
  
  switch(accel.getDataRate()) {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print("0.10 ");
      break;
    default:
      Serial.print("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void) {
  Serial.print("Range:         +/- ");
  
  switch(accel.getRange()) {
    case ADXL345_RANGE_16_G:
      Serial.print("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print("2 ");
      break;
    default:
      Serial.print("?? ");
      break;
  }
  Serial.println(" g");
}

void sendSensorData() {
  // Configure the peer (requester) dynamically
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, requesterMAC, 6);
  peerInfo.channel = 0; // Default channel (adjust if needed)
  peerInfo.encrypt = false;

  // Add the peer if not already added
  if (!esp_now_is_peer_exist(requesterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  // Collect and store the sensor data
  for (int count = 0; count < TAM; count++) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    myData[count].ax = event.acceleration.x;
    myData[count].ay = event.acceleration.y;
    myData[count].az = event.acceleration.z;
    myData[count].distance = distance;
    myData[count].tempo = millis() - timestamp;

    // Print the collected data for debugging
    Serial.print("Data ");
    Serial.print(count);
    Serial.print(": ax=");
    Serial.print(myData[count].ax);
    Serial.print(", ay=");
    Serial.print(myData[count].ay);
    Serial.print(", az=");
    Serial.print(myData[count].az);
    Serial.print(", distance=");
    Serial.print(myData[count].distance);
    Serial.print(", tempo=");
    Serial.println(myData[count].tempo);
  }

  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(requesterMAC, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent sensor data successfully");
  } else {
    Serial.printf("Error sending sensor data: %d\n", result);
  }

  // Remove the peer after sending to free memory
  esp_now_del_peer(requesterMAC);
}

void setup() {
  Serial.begin(115200);

  Serial.println("Starting setup...");
  WiFi.mode(WIFI_STA);
    mySerial.begin(9600); // RX: GPIO 4, TX: GPIO 5
  Serial.println("WiFi Mode set to STA.");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized");

  // Register the callback to receive data
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW started and callback registered");

  Serial.println("Accelerometer Test");
  Serial.println("");
 
  /* Initialize the sensor */
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  
  displayDataRate();
  displayRange();
  Serial.println("");

  setupTimer(); // Set up the timer interrupt
  Serial.println("Timer initialized.");
}

void loop() {
  // If the flag is set, attempt to read from UART and send data
  if (readUARTFlag) {
    readUARTFlag = false; // Reset the flag
    
    if (mySerial.available() >= 4) {
      for (int i = 0; i < 4; i++) {
        data[i] = mySerial.read();
      }

      if (data[0] == 0xff) {
        int sum = (data[0] + data[1] + data[2]) & 0x00FF;
        if (sum == data[3]) {
          distance = (data[1] << 8) + data[2];
          if (distance > 30) {
            Serial.print("distance=");
            Serial.print(distance);
            Serial.println("mm");
          } else {
            Serial.println("Below the lower limit");
          }
        } else {
          Serial.println("Checksum error");
        }
      }
    }

    // If we have received a signal to send data, do so
    if (sendDataFlag) {
      sendSensorData();
      sendDataFlag = false;
    }
  }
}
