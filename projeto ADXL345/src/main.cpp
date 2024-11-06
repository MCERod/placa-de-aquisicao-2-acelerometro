
/*#include <Arduino.h>
#include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_ADXL345_U.h>

 /* Assign a unique ID to this sensor at the same time */
 /*Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

 void displaySensorDetails(void)
 {
   sensor_t sensor;
   accel.getSensor(&sensor);
   Serial.println("------------------------------------");
   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
   Serial.println("------------------------------------");
   Serial.println("");
   delay(500);
 }

 void displayDataRate(void)
 {
   Serial.print  ("Data Rate:    "); 
  
   switch(accel.getDataRate())
   {
     case ADXL345_DATARATE_3200_HZ:
       Serial.print  ("3200 "); 
        break;
     case ADXL345_DATARATE_1600_HZ:
       Serial.print  ("1600 "); 
       break;
     case ADXL345_DATARATE_800_HZ:
       Serial.print  ("800 "); 
       break;
     case ADXL345_DATARATE_400_HZ:
       Serial.print  ("400 "); 
       break;
     case ADXL345_DATARATE_200_HZ:
       Serial.print  ("200 "); 
       break;
     case ADXL345_DATARATE_100_HZ:
       Serial.print  ("100 "); 
       break;
     case ADXL345_DATARATE_50_HZ:
        Serial.print  ("50 "); 
       break;
     case ADXL345_DATARATE_25_HZ:
       Serial.print  ("25 "); 
       break;
     case ADXL345_DATARATE_12_5_HZ:
       Serial.print  ("12.5 "); 
       break;
     case ADXL345_DATARATE_6_25HZ:
       Serial.print  ("6.25 "); 
       break;
     case ADXL345_DATARATE_3_13_HZ:
       Serial.print  ("3.13 "); 
       break;
     case ADXL345_DATARATE_1_56_HZ:
       Serial.print  ("1.56 "); 
       break;
     case ADXL345_DATARATE_0_78_HZ:
       Serial.print  ("0.78 "); 
       break;
     case ADXL345_DATARATE_0_39_HZ:
       Serial.print  ("0.39 "); 
       break;
     case ADXL345_DATARATE_0_20_HZ:
       Serial.print  ("0.20 "); 
        break;
     case ADXL345_DATARATE_0_10_HZ:
       Serial.print  ("0.10 "); 
       break;
      default:
       Serial.print  ("???? "); 
       break;
   }  
   Serial.println(" Hz");  
 }

 void displayRange(void)
 {
   Serial.print  ("Range:         +/- "); 
  
   switch(accel.getRange())
   {
     case ADXL345_RANGE_16_G:
       Serial.print  ("16 "); 
       break;
     case ADXL345_RANGE_8_G:
       Serial.print  ("8 "); 
       break;
     case ADXL345_RANGE_4_G:
       Serial.print  ("4 "); 
       break;
     case ADXL345_RANGE_2_G:
       Serial.print  ("2 "); 
       break;
     default:
       Serial.print  ("?? "); 
       break;
   }  
   Serial.println(" g");  
 }

 void setup(void) 
 {

   Serial.begin(9600);
   Serial.println("Accelerometer Test"); Serial.println("");
 */
   /* Initialise the sensor */
  /* if(!accel.begin())
   {
     /* There was a problem detecting the ADXL345 ... check your connections */
    /* Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
     while(1);
   }
*/
   /* Set the range to whatever is appropriate for your project */
 /*  accel.setRange(ADXL345_RANGE_16_G);
   // accel.setRange(ADXL345_RANGE_8_G);
   // accel.setRange(ADXL345_RANGE_4_G);
   // accel.setRange(ADXL345_RANGE_2_G);
  */
    /* Display some basic information on this sensor */
 /*  displaySensorDetails();

   /* Display additional settings (outside the scope of sensor_t) */
 /*  displayDataRate();
   displayRange();
   Serial.println("");
 }

 void loop(void) 
 {*/
   /* Get a new sensor event */ 
  /* sensors_event_t event; 
   accel.getEvent(&event);
 */
   /* Display the results (acceleration is measured in m/s^2) */
  /* Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
   delay(500);
 }
 */
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX: GPIO 4, TX: GPIO 5

#define TAM 8 // Limite de dados a serem enviados no máximo 8, um array de 8

// Endereço do solicitante
uint8_t requesterMAC[6];

// ADXL345 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
unsigned long timestamp = 0;
int distance;
unsigned char data[4]; // Array to store incoming serial data

// Estrutura para armazenar dados do sensor
typedef struct struct_message {
  float ax;
  float ay;
  float az;
  int distance;
  int tempo;
} struct_message;

// Array de estruturas struct_message para armazenar os dados
struct_message myData[TAM];

// Flag para indicar quando enviar dados
bool sendDataFlag = false;

void printMAC(const uint8_t *mac, const char *label) {
  Serial.print(label);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// Callback para receber dados e ativar o envio
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.println("Received signal to send data");
  memcpy(requesterMAC, mac_addr, 6); // Armazena o MAC do solicitante
  printMAC(requesterMAC, "Responder para: ");
  sendDataFlag = true;
  timestamp = millis();
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
  // Configura o peer (solicitante) dinamicamente
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, requesterMAC, 6);
  peerInfo.channel = 0; // Canal padrão (ajuste se necessário)
  peerInfo.encrypt = false;

  // Adiciona o peer se ainda não foi adicionado
  if (!esp_now_is_peer_exist(requesterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  // Coleta e armazena os dados do sensor
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

    // delay(100); // Ajuste o delay conforme necessário
  }

  // Envia os dados via ESP-NOW
  esp_err_t result = esp_now_send(requesterMAC, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent sensor data successfully");
  } else {
    Serial.printf("Error sending sensor data: %d\n", result);
  }

  // Remove o peer após o envio para liberar memória
  esp_now_del_peer(requesterMAC);
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600); // RX: GPIO 4, TX: GPIO 5
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Registra o callback para receber dados
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW iniciado e callback registrado");

  Serial.println("Accelerometer Test");
  Serial.println("");
 
  /* Initialise the sensor */
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displayDataRate();
  displayRange();
  Serial.println("");
}

void loop() {
  // Envia os dados se o flag estiver ativado
  if (sendDataFlag) {
    // Read data from the serial port
    if (mySerial.available() >= 4) {
      for (int i = 0; i < 4; i++) {
        data[i] = mySerial.read();
      }

      // Check if the first byte is 0xff
      if (data[0] == 0xff) {
        int sum = (data[0] + data[1] + data[2]) & 0x00FF;
        if (sum == data[3]) {
          distance = (data[1] << 8) + data[2];
          if (distance > 30) {
            // Print the distance in millimeters
            Serial.print("distance=");
            Serial.print(distance);
            Serial.println("mm");
          } else {
            Serial.println("Below the lower limit");
          }
        } else {
          Serial.println("ERROR: Checksum mismatch");
        }
      } else {
        Serial.println("ERROR: Invalid start byte");
      }
    } else {
      Serial.println("ERROR: Not enough data available");
    }
    sendSensorData();
    sendDataFlag = false;
  }
}