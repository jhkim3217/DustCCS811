#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include <HardwareSerial.h>
#include <WiFiClientSecure.h>

//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address
#define PIN_NOT_WAKE 5

// PM2.5 / PM10
// #define DEBUG   
#define  MEAN_NUMBER    10
#define  MAX_PM         0
#define  MIN_PM         32767

CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;
WiFiClientSecure client;

//--------PM Start
HardwareSerial Serial1(2);  // Hardware Serial Pin Number is already defined.
int incomingByte = 0; // for incoming serial data
const int MAX_FRAME_LEN = 64;
char frameBuf[MAX_FRAME_LEN];
int detectOff = 0;
int frameLen = MAX_FRAME_LEN;
bool inFrame = false;
char printbuf[256];
unsigned int calcChecksum = 0;
unsigned int pm1_0=0, pm2_5=0, pm10_0=0;
unsigned int tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0; 
unsigned int tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0; 
byte i=0;
struct PMS7003_framestruct {
    byte  frameHeader[2];
    unsigned int  frameLen = MAX_FRAME_LEN;
    unsigned int  concPM1_0_CF1;
    unsigned int  concPM2_5_CF1;
    unsigned int  concPM10_0_CF1;
    unsigned int  checksum;
} thisFrame;
//--------PM End

const char* ssid     = "RecursiveSoft";
const char* password = "abcde67890";
const char* server = "openfunction.recursivesoft.net";  // Server URL
const char* test_root_ca= \
"-----BEGIN CERTIFICATE-----\n" \
"MIID0jCCArqgAwIBAgIJALRri7Z/OGMFMA0GCSqGSIb3DQEBCwUAMIGbMRwwGgYD\n" \
"VQQDDBMqLnJlY3Vyc2l2ZXNvZnQubmV0MQswCQYDVQQGEwJLUjEOMAwGA1UEBwwF\n" \
"QnVzYW4xGjAYBgNVBAoMEVJlY3Vyc2l2ZVNvZnQgSW5jMREwDwYDVQQLDAhEZXYg\n" \
"VGVhbTEOMAwGA1UECAwFQnVzYW4xHzAdBgkqhkiG9w0BCQEWEGtuaWdocEBnbWFp\n" \
"bC5jb20wHhcNMTcwOTI0MTEyODI1WhcNMjcwOTIyMTEyODI1WjCBmzEcMBoGA1UE\n" \
"AwwTKi5yZWN1cnNpdmVzb2Z0Lm5ldDELMAkGA1UEBhMCS1IxDjAMBgNVBAcMBUJ1\n" \
"c2FuMRowGAYDVQQKDBFSZWN1cnNpdmVTb2Z0IEluYzERMA8GA1UECwwIRGV2IFRl\n" \
"YW0xDjAMBgNVBAgMBUJ1c2FuMR8wHQYJKoZIhvcNAQkBFhBrbmlnaHBAZ21haWwu\n" \
"Y29tMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAkkTJ9f3w93rAnDSa\n" \
"POJS03UQ7C1Hw8sVIvHhKHG8cQ9xKbxsSzTRRXrtwCjZPAW0W5pkyHnbejzPgIty\n" \
"UaPj7rnl6jwD5tYObA5CZ0wKX7is1dliz1uUVlplKqDfUK6WzY6jcrAAl0MCfeNH\n" \
"OFA3OXmbj+x+srCQwPXYtkY2+H5xcjzvyRnlRArDWOJpxwiKvm1uQF/fY7x3CfEs\n" \
"7gIM3LiN+peMTvwIplGjWA8x1aAoegZ4dwGn/js00128HDfnnAdWxSNo4yNiyrlj\n" \
"6HE9n7nvUsRdPuDLvJTCmqv7CiT1yHjtWA6MzWV93dp7U37WzfyQ7qnt34cb7rpq\n" \
"6uVR2wIDAQABoxcwFTATBgNVHSUEDDAKBggrBgEFBQcDATANBgkqhkiG9w0BAQsF\n" \
"AAOCAQEAdYmoGuS9eHkk0sg/ZdPh7zodFLifK3sJMiCO9rHGDrMIZL92nye6oPuP\n" \
"xnBI9+GpzuZWZ190RNx/55ErasCb9YQvZI0FOObGh95ajgOif71HHvTiE5yimBO8\n" \
"z0KPPAlEDW1dZCb724EEgeIO964U0+me8G5E2JdBrTg76egSSdNlY7HDR+jzZTc1\n" \
"ccbRThLODWdKGDcYw+t9hAT1ERrY+tGRwA7R91+5mNeQ0a+PKj+BRTMGwAvkiSXp\n" \
"n6P+C/11PjhdwaKFTuZ98Bhk0YBL4L/aoor0bT0SHHkTEF62cOSJJ9RDV4TH/lUH\n" \
"ivOjrkpd4Nl69PaTAf2uZkdHKrfavg==\n" \
"-----END CERTIFICATE-----\n";      

// https://openfunction.azurewebsites.net/api/DustWebhookJS?code=CCdQO07oIaSwHC20M6jwqfIFMLo8BVaN5S1aHqdG/cHWWNIaYKabng==&clientId=default

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
//  Serial1.begin(9600); // UART with pm2.5/pm10 sensor module
  
  delay(100);
  Serial.println("Wifi / BME280 / CCS811");
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  // Start WIFI
  WiFi.begin(ssid, password);

  // Attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting...");
    delay(1000);
  }  
  
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // Get NTP DateTime ============================
  Serial.print("Setting time using SNTP");
  
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  while (now < 1000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
  
    //This begins the CCS811 sensor and prints error status of .begin()
  CCS811Core::status returnCode = myCCS811.begin();
  Serial.print("CCS811 begin exited with: ");
  //Pass the error code to a function to print the results
  // TODO: printDriverError( returnCode );
  Serial.println();

  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;

  //Initialize BME280
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  myBME280.begin();
}

// PM

bool pms7003_read() {
#ifdef DEBUG  
    Serial.println("----- Reading PMS7003 -----");
#endif
    Serial1.begin(9600);
    bool packetReceived = false;
    calcChecksum = 0;

    while (!packetReceived) {
        if (Serial1.available() > 32) {
            int drain = Serial1.available();
#ifdef DEBUG
                Serial.print("----- Draining buffer: -----");
                Serial.println(Serial1.available(), DEC);
#endif

            for (int i = drain; i > 0; i--) {
                Serial1.read();
            }
        }
        if (Serial1.available() > 0) {
#ifdef DEBUG
                Serial.print("----- Available: -----");
                Serial.println(Serial1.available(), DEC);
#endif
            incomingByte = Serial1.read();
#ifdef DEBUG
                Serial.print("----- READ: -----");
                Serial.println(incomingByte, HEX);
#endif
            if (!inFrame) {
                if (incomingByte == 0x42 && detectOff == 0) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame.frameHeader[0] = incomingByte;
                    calcChecksum = incomingByte; // Checksum init!
                    detectOff++;
                }
                else if (incomingByte == 0x4D && detectOff == 1) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame.frameHeader[1] = incomingByte;
                    calcChecksum += incomingByte;
                    inFrame = true;
                    detectOff++;
                }
                else {
                    Serial.print("----- Frame syncing... -----");
                    Serial.print(incomingByte, HEX);
                    Serial.println();
                }
            }
            else {
                frameBuf[detectOff] = incomingByte;
                calcChecksum += incomingByte;
                detectOff++;
                unsigned int  val = (frameBuf[detectOff-1]&0xff)+(frameBuf[detectOff-2]<<8);
                switch (detectOff) {
                    case 4:
                        thisFrame.frameLen = val;
                        frameLen = val + detectOff;
                        break;
                    case 6:
                        thisFrame.concPM1_0_CF1 = val;
                        break;
                    case 8:
                        thisFrame.concPM2_5_CF1 = val;
                        break;
                    case 10:
                        thisFrame.concPM10_0_CF1 = val;
                        break;
                    case 32:
                        thisFrame.checksum = val;
                        calcChecksum -= ((val>>8)+(val&0xFF));
                        break;
                    default:
                        break;
                }

                if (detectOff >= frameLen) {
#ifdef DEBUG          
                    sprintf(printbuf, "PMS7003 ");
                    sprintf(printbuf, "%s[%02x %02x] (%04x) ", printbuf,
                        thisFrame.frameHeader[0], thisFrame.frameHeader[1], thisFrame.frameLen);
                    sprintf(printbuf, "%sCF1=[%04x %04x %04x] ", printbuf,
                        thisFrame.concPM1_0_CF1, thisFrame.concPM2_5_CF1, thisFrame.concPM10_0_CF1);
                    sprintf(printbuf, "%scsum=%04x %s xsum=%04x", printbuf,
                        thisFrame.checksum, (calcChecksum == thisFrame.checksum ? "==" : "!="), calcChecksum);
                    Serial.println(printbuf);
#endif        
                    packetReceived = true;
                    detectOff = 0;
                    inFrame = false;
                }
            }
        }
    }
    Serial1.end();
    return (calcChecksum == thisFrame.checksum);
}

int max(int numA, int numB) {
  return numA > numB ? numA : numB;
}

int min(int numA, int numB) {
  return numA < numB ? numA : numB;
}


void loop() {

  // PM Start--------------------
    if(i==0) { 
    tmp_max_pm1_0  = MAX_PM;
    tmp_max_pm2_5  = MAX_PM;
    tmp_max_pm10_0 = MAX_PM;
    tmp_min_pm1_0  = MIN_PM;
    tmp_min_pm2_5  = MIN_PM;
    tmp_min_pm10_0 = MIN_PM;
  }
  if (pms7003_read()) {
    tmp_max_pm1_0  = max(thisFrame.concPM1_0_CF1, tmp_max_pm1_0);
    tmp_max_pm2_5  = max(thisFrame.concPM2_5_CF1, tmp_max_pm2_5);
    tmp_max_pm10_0 = max(thisFrame.concPM10_0_CF1, tmp_max_pm10_0);
    tmp_min_pm1_0  = min(thisFrame.concPM1_0_CF1, tmp_min_pm1_0);
    tmp_min_pm2_5  = min(thisFrame.concPM2_5_CF1, tmp_min_pm2_5);
    tmp_min_pm10_0 = min(thisFrame.concPM10_0_CF1, tmp_min_pm10_0);
    pm1_0 += thisFrame.concPM1_0_CF1;
    pm2_5 += thisFrame.concPM2_5_CF1;
    pm10_0 += thisFrame.concPM10_0_CF1;
    i++;
    Serial.print("O");

  }
  else {
    Serial.print("*");
  }
  
  if(i==MEAN_NUMBER) {
  sprintf(printbuf, "[Checksum OK]");
  sprintf(printbuf, "%s PM1.0 = %02x, PM2.5 = %02x, PM10 = %02x", printbuf, 
    (pm1_0-tmp_max_pm1_0-tmp_min_pm1_0)/(MEAN_NUMBER-2), 
    (pm2_5-tmp_max_pm2_5-tmp_min_pm2_5)/(MEAN_NUMBER-2), 
    (pm10_0-tmp_max_pm10_0-tmp_min_pm10_0)/(MEAN_NUMBER-2));
  sprintf(printbuf, "%s [max =%02x,%02x,%02x, min = %02x,%02x,%02x]", printbuf,
    tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0,
    tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0);  
  Serial.println();
  Serial.println(printbuf);
  pm1_0=pm2_5=pm10_0=i=0;
//  delay(20000);
  }
  // PM End--------------------
  
  if (myCCS811.dataAvailable()) {
  
    myCCS811.readAlgorithmResults();
    float BMEtempC = myBME280.readTempC();
    float BMEhumid = myBME280.readFloatHumidity();
    myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
  
  } else if (myCCS811.checkForStatusError()) {
    printSensorError();
  }

  // Send Data to Server
  client.setCACert(test_root_ca);
  Serial.println("\nStarting connection to server...");
  
  if (!client.connect(server, 443))
    Serial.println("Connection failed!");
  else {
    Serial.println("Connected to server!");  
    client.println("POST /api/DustWebHookJS?code=CCdQO07oIaSwHC20M6jwqfIFMLo8BVaN5S1aHqdG/cHWWNIaYKabng==&clientId=default HTTP/1.0");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");       
  }

  // Data Transfer
  client.println("Content-Type: application/json");
//  String postData = "{\"id\":\"96\", \"temp\":\"15\", \"humi\":\"20\", \"co2\":\"500\", \"voc\":\"2\", \"pm25\":\"25\", \"pm10\":\"10\"}";
  String postData = "{\"id\":";
  postData += "\"96\","; 
  postData += "\"temp\":";
  postData += myBME280.readTempC();
  postData += ",";
  postData += "\"humi\":";
  postData += myBME280.readFloatHumidity();
  postData += ",";
  postData += "\"co2\":";
  postData += myCCS811.getCO2();
  postData += ",";
  postData += "\"voc\":";
  postData += myCCS811.getTVOC();
  postData += ",";
  postData += "\"pm25\":";
  postData += "\"25\",";
  postData += "\"pm10\":";
  postData += "\"10\"";
  postData += "}";
    
  client.print("Content-Length: ");
  client.println(postData.length());
  client.println("");
  client.println(postData);
    
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  // if there are incoming bytes available
  // from the server, read them and print them:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  client.stop();  
  delay(100000);  // every 100 seconds
}


//printSensorError gets, clears, then prints the errors
//saved within the error register.
void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
  } else {
    Serial.print("Error: ");
    if (error & 1 << 5) Serial.print("HeaterSupply");
    if (error & 1 << 4) Serial.print("HeaterFault");
    if (error & 1 << 3) Serial.print("MaxResistance");
    if (error & 1 << 2) Serial.print("MeasModeInvalid");
    if (error & 1 << 1) Serial.print("ReadRegInvalid");
    if (error & 1 << 0) Serial.print("MsgInvalid");
    Serial.println();
  }
}
