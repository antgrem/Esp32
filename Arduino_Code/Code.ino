#include <WiFi.h>
#include <PCD8544.h>
#include "driver/gpio.h"
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "esp_system.h"
  
//DS18B20 libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// Define CS pin for the SD card module
#define SD_CS 5

TaskHandle_t Task1;
TaskHandle_t Task2;
xQueueHandle xQueue;

String dataMessage;

/* structure that hold data*/
typedef struct{
  int sender;
  float msg;
  int adc;
  int val;
}Data;

const char* ssid     = "****";
const char* password = "****";
static PCD8544 lcd=PCD8544(14,13,27,26,15);
//static const char ntpServerName[] = "time.nist.gov";
static const char ntpServerName[] = "2.pool.ntp.org";
const int timeZone = 1;
WiFiUDP Udp;
unsigned int localPort = 8888;
time_t getNtpTime();
void getReadings();

char str[50];

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;
// variable for storing the potentiometer value
int potValue = 0;

// Data wire is connected to ESP32 GPIO 21
#define ONE_WIRE_BUS 21
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Temperature Sensor variables
float temperature;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void setup()
{
  char count=0;
  File root;
  
    Serial.begin(115200);
    Serial.print("Start to work");
    gpio_set_direction( GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_set_level( GPIO_NUM_23, 1);
    WiFi.begin(ssid, password);
    lcd.begin(84, 48);

  // Start the DallasTemperature library
  sensors.begin(); 

  
  // Initialize SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }

  root = SD.open("/");

  printDirectory(root, 0);

  Serial.println("done!");

    // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();


  getReadings();
  lcd.setCursor(0, 5);
  lcd.print("t'=");
  lcd.print(temperature);

        delay(200);
        delay(200);
        delay(200);
        delay(200);
        delay(200);
    while (WiFi.status() != WL_CONNECTED) 
    {
    // lcd.clear();
      lcd.setCursor(0, 5);
      getReadings();
      lcd.print("t'=");
      lcd.print(temperature);

      lcd.setCursor(1, 0);
      lcd.print("             ");

      lcd.setCursor(1, 0);
      lcd.print("Connecting");
      delay(200);
        lcd.print(".");
      delay(400);
        lcd.print(".");
      delay(400);   
      lcd.print(".");
      delay(400);
      if (count++ == 10)
        break;
    }


  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Connected to WiFi ");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.setCursor(0, 3);
  lcd.println(WiFi.localIP());
  delay(400);
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Waiting for Time Sync.. ");
  delay(500);
  Udp.begin(localPort);

  disableCore0WDT();
  disableCore1WDT();

  /* create the queue which size can contains 5 elements of Data */
  xQueue = xQueueCreate(5, sizeof(Data));

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

time_t prevDisplay = 0;

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters )
{
  /* keep the status of sending data */
  BaseType_t xStatus;
  /* time to block the task until data is available */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  /* create data to send */
  Data data;
  /* sender 1 has id is 1 */
  data.sender = 1;

  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {
    getReadings();
    data.val = hallRead();
   // Reading potentiometer value
   data.adc = analogRead(potPin);

    data.msg = temperature;
    /* send data to front of the queue */
    xStatus = xQueueSendToFront( xQueue, &data, xTicksToWait );
    /* check whether sending is ok or not */
//    if( xStatus == pdPASS ) 
//    {
//      /* increase counter of sender 1 */
//      Serial.println("sendTask sent data");
//    }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(temperature).c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(temperature).c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(temperature).c_str());
  });

  // Start server
  server.begin();

  //Serial.println("\n\nLife after server begin \n\n");

    delay(200);
  }
    vTaskDelete( NULL );
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters )
{
  /* keep the status of receiving data */
  BaseType_t xStatus;
  /* time to block the task until data is available */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  Data data;
  float tempr;

  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    /* receive data from the queue */
    xStatus = xQueueReceive( xQueue, &data, xTicksToWait );
    /* check whether receiving is ok or not */
    if(xStatus == pdPASS){
//      Serial.print("receiveTask run on core ");
      /* get the core that the task was pinned to */
//      Serial.print(xTaskGetAffinity(Task2));
      /* print the data to terminal */
//      Serial.print(" got data: ");
//      Serial.print("sender = ");
//      Serial.print(data.sender);
//      Serial.print(" msg = ");
//      Serial.println(data.msg);
      tempr = data.msg;
      potValue = data.adc;

//      dataMessage = itoa(tempr) + " " + itoa(potValue) + "\r\n";
      sprintf(str, "%d:%d %f %d \r\n", hour(), minute(), tempr, potValue);
//      logSDCard();
      
    }

    
   if (timeStatus() != timeNotSet)  
    {
     if (now() != prevDisplay) { //update the display only if time has changed
       prevDisplay = now();
       lcd.clear();
       digitalClockDisplay();
     }
    }

  vTaskDelay(10);


    lcd.setCursor(0, 5);
    lcd.print("t'=");
    lcd.print(tempr);
    lcd.setCursor(0, 4);
    lcd.print(potValue);
    Serial.print(data.val);
    Serial.println(touchRead(4));  // get value of Touch 0 pin = GPIO 4
    
    delay(300);
  }

  vTaskDelete( NULL );
}

void loop() {
  
}

// void loop() {
//   getReadings();
//   // Reading potentiometer value
//   potValue = analogRead(potPin);
//   Serial.println(potValue);

//    if (timeStatus() != timeNotSet)  
//    {
//     if (now() != prevDisplay) { //update the display only if time has changed
//       prevDisplay = now();
//       digitalClockDisplay();
//     }
//    }
        
//     lcd.setCursor(0, 5);
//     lcd.print("t'=");
//     lcd.print(temperature);
//     lcd.setCursor(0, 4);
//     lcd.print(potValue);
    
// }
void digitalClockDisplay()
{
//  lcd.clear();
  // digital clock display of the time
  lcd.setCursor(20,1);
  lcd.print(hour());
  printDigits(minute());
  printDigits(second());
  lcd.setCursor(20,3);
  lcd.print(day());
  lcd.print(".");
  lcd.print(month());
  lcd.print(".");
  lcd.print(year());
 }

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if (digits < 10)
  lcd.print('0');
  lcd.print(digits);
 }

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
   // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

// Function to get temperature
void getReadings(){
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0); // Temperature in Celsius
  //temperature = sensors.getTempFByIndex(0); // Temperature in Fahrenheit
  // Serial.print("Temperature: ");
  // Serial.println(temperature);
}


// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Write the sensor readings on the SD card
void logSDCard() {
  //dataMessage = "test";//String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," + 
                //String(temperature) + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  //appendFile(SD, "/data.txt", dataMessage.c_str());
  appendFile(SD, "/data.txt", str);
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
