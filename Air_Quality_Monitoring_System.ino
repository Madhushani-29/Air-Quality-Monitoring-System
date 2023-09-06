#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <FirebaseArduino.h>      
#include <DHT.h> 
#include <MQUnifiedsensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define WIFI_SSID "HUAWEI nova Y61" 
#define WIFI_PASSWORD "12345678"  
#define FIREBASE_HOST "airqualitymonitoringsyst-3dbcd-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "BEjZChjJXbDHzoRRRkMsoVwfUwWbdojhXxhPAdJr" 
#define DHTPIN D4                                            
#define DHTTYPE DHT11 
#define Board ("ESP8266")
#define Pin  (A0)  
#define Type ("MQ135")
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (10)
#define RatioMQ135CleanAir (3.59)

#define CO2Threshold 1000
#define COThreshold 9
#define NH4Threshold 4

#define ledPin D3

MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

DHT dht(DHTPIN, DHTTYPE); 

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);

  dht.begin();                                                  //reads dht sensor data 
               
  lcd.begin();

  pinMode(ledPin, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                                  
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  Serial.println();
  Serial.print("Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());                               //prints local IP address
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);  

  setSyncProvider(getNtpTime);
  setSyncInterval(600); // Sync time every 10 minutes

  timeClient.begin();
  timeClient.setTimeOffset(19800); // Set your time zone offset in seconds

  MQ135.setRegressionMethod(1);
  MQ135.init();
  MQ135.setRL(10);
  Serial.print("Calibrating, please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println(" done!.");
  
  if (isinf(calcR0) || calcR0 == 0) {
    Serial.println("***** Values from MQ-135 ********");
  }  
}

void loop() {
  MQ135.update();

  MQ135.setA(76.63); MQ135.setB(-2.64);
  float CO2 = MQ135.readSensor();

  MQ135.setA(315.63); MQ135.setB(-2.81);
  float CO = MQ135.readSensor();

  MQ135.setA(102.4); MQ135.setB(-2.78);
  float NH4 = MQ135.readSensor(); 

  float h = dht.readHumidity();                                 // Read Humidity
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))                                     // Checking sensor working
  {                                   
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  else if (isnan(CO2) || isnan(CO) || isnan(NH4))                                     // Checking sensor working
  {                                   
    Serial.println(F("Failed to read from MQ2 sensor!"));
    return;
  } 

  else{
    displaySensorData(h, t, CO2, CO, NH4);
    displayAirQuality(CO2, CO, NH4);
    
    if (timeStatus() == timeSet) {
      if (Firebase.failed()) 
      {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error()); 
        return;
      }
      else{
        saveDataPermenently(year(), month(), day(), hour(timeClient.getEpochTime()), h, t, CO2, CO, NH4);
        saveDataRealtimeDatabase(h, t, CO2, CO, NH4);
      }
    }
  }
  delay(1000);
}

void displaySensorData(float h, float t, float CO2, float CO, float NH4){
  Serial.print("Humidity: ");  
  Serial.print(h);
  String fireHumid = String(h) + String("%");                   //Humidity integer to string conversion
  
  Serial.print("%  Temperature: ");  
  Serial.print(t);  
  Serial.println("°C ");
  String fireTemp = String(t) + String("°C");                  //Temperature integer to string conversion
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature: " + fireTemp);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + fireHumid);
  delay(2000);

  Serial.println("|    CO2     |    CO      |    NH4     |");

  Serial.print("|    "); Serial.print(CO2);
  Serial.print("    |    "); Serial.print(CO);
  Serial.print("    |    "); Serial.print(NH4);
  Serial.println("    |");

  String CO2Composition = String(CO2);
  String COComposition = String(CO);
  String NH4Composition = String(NH4);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: " +  CO2Composition);
  lcd.setCursor(0, 1);
  lcd.print("CO: " + COComposition);
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("NH4: " + NH4Composition);
  delay(2000);
}
  
void saveDataPermenently(int year, int month, int day, int hour, float h, float t, float CO2, float CO, float NH4) {
  String location="Colombo";
  String commonPathDHT=String("/Database/")+String(location)+String("/")+String(year)+String("-")+String(month)+String("-")+String(day)+String("/")+String(hour)+String("/Temp_Hum/");
  String commonPathMQ2=String("/Database/")+String(location)+String("/")+String(year)+String("-")+String(month)+String("-")+String(day)+String("/")+String(hour)+String("/Gas_Composition/");

  String setHumPath=String(commonPathDHT)+String("Humidity");
  String setTempPath=String(commonPathDHT)+String("Temperature");
  String setCO2Path=String(commonPathMQ2)+String("CO2");
  String setCOPath=String(commonPathMQ2)+String("CO");
  String setNH4Path=String(commonPathMQ2)+String("NH4");
  
  Firebase.setFloat(setHumPath, h);
  Firebase.setFloat(setTempPath,t);
  Firebase.setFloat(setCO2Path,CO2);
  Firebase.setFloat(setCOPath,CO);
  Firebase.setFloat(setNH4Path,NH4);
}

void saveDataRealtimeDatabase(float h, float t, float CO2, float CO, float NH4) {
  String location="Colombo";
  String commonPathDHT=String("/RealtimeDatabase/")+String(location)+String("/")+String("/Temp_Hum/");
  String commonPathMQ2=String("/RealtimeDatabase/")+String(location)+String("/Gas_Composition/");

  String setHumPath=String(commonPathDHT)+String("Humidity");
  String setTempPath=String(commonPathDHT)+String("Temperature");
  String setCO2Path=String(commonPathMQ2)+String("CO2");
  String setCOPath=String(commonPathMQ2)+String("CO");
  String setNH4Path=String(commonPathMQ2)+String("NH4");

  Firebase.setFloat(setHumPath, h);
  Firebase.setFloat(setTempPath,t);
  Firebase.setFloat(setCO2Path,CO2);
  Firebase.setFloat(setCOPath,CO);
  Firebase.setFloat(setNH4Path,NH4);
}

void displayAirQuality(float CO2, float CO, float NH4){
  if (CO2 > CO2Threshold || CO > COThreshold || NH4 > NH4Threshold) {
    if (CO2 > CO2Threshold) {
      Serial.println("CO2 is High.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CO2 is High");
      delay(2000);
    }
    if (CO > COThreshold) {
      Serial.println("CO is High.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CO is High");
      delay(2000);
    }
    if (NH4 > NH4Threshold) {
      Serial.println("NH4 is High.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("NH4 is High");
      delay(2000);
    }
  } else {
    Serial.println("Fresh Air");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fresh Air");
    delay(2000);
  }
}

time_t getNtpTime() {
  timeClient.update();
  return timeClient.getEpochTime();
}
