
#include <ESP8266WiFi.h>                      // Подключаем библиотеку ESP8266WiFi
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
 
const char* ssid = "SSID";     // Название Вашей WiFi сети
const char* password = "password";// Пароль от Вашей WiFi сети

#define RELAY_IP "192.168.1.75" 

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

//#define LED 1                                 // GPIO1/TXD01 - светодиод
 
void setup(){
  delay(2200);      
  Serial.begin(115200);                       // Скорость передачи 115200 
    // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  //pinMode(LED,OUTPUT);                        // Указываем вывод RELAY как выход
  //digitalWrite(LED, HIGH);                    // Устанавливаем LED в HIGH
  Serial.println();                           // Печать пустой строки 
  Serial.print("Connecting to ");             // Печать "Подключение к:"
  Serial.println(ssid);                       // Печать "Название Вашей WiFi сети"

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);                 // Подключение к WiFi Сети
 
  while (WiFi.status() != WL_CONNECTED)       // Проверка подключения к WiFi сети
  { 
    delay(500);                               // Пауза 500 мс
    Serial.print(".");                        // Печать "."
    switch (WiFi.status()){
      case WL_NO_SSID_AVAIL: {
        Serial.println("WiFi not available");
        delay(1500);   
        continue;
      }; 
      case WL_CONNECT_FAILED: {
        Serial.println("WiFi pass failed");
        delay(1500);   
        continue;
      }; 
    };
  }
  Serial.println("");                         // Печать пустой строки  
  Serial.println("WiFi connected");           // Печать "WiFi connected"
 
//  server.begin();                             // Запуск сервера
//  Serial.println("Server started");           // Печать "Server starte"
  Serial.print("Local IP: ");  // Печать "Use this URL to connect:" 
  Serial.print(WiFi.localIP());               // Печать выданого IP адресса          
}
 
void loop(){
  if (!WiFi.isConnected()) {
    ESP.restart();
  };

  WiFiClient client;
  HTTPClient http;

    if (stringComplete) {
      //Serial.println(inputString);
      
      if (inputString.indexOf("AT") != -1)  { // обрабатываем команду AT для проверки связи с модулем
         Serial.println("OK");
      }
      
      if (inputString.indexOf("PING") != -1)  { // обрабатываем команду PING для проверки связи с релейным модулем
        http.setTimeout(3000);
        if (http.begin(client, "http://" RELAY_IP)) {  // HTTP
          // start connection and send HTTP header
          int httpCode = http.GET();
          // httpCode will be negative on error
          if (httpCode > 0) { 
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
               Serial.println("OK");
            } else {
               Serial.printf("ERROR 2");
            }
          } else {
            Serial.println("ERROR 1");
          }
          http.end();
        } else {
          Serial.printf("ERROR 0");
        }
      }

      if (inputString.indexOf("RESET") != -1)  { // обрабатываем команду RESET 
        http.setTimeout(15000); // ответ придёт только после ресета
        if (http.begin(client, "http://" RELAY_IP "/RELAY=RESET")) {  // HTTP
          // start connection and send HTTP header
          int httpCode = http.GET();
          // httpCode will be negative on error
          if (httpCode > 0) { 
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
               String payload = http.getString();
               if (payload.indexOf("RESET OK") != -1) { 
                  Serial.println("OK");
               } else {
                  Serial.printf("ERROR 2");
               }
            }
          } else {
            Serial.println("ERROR 1");
          }
          http.end();
        } else {
          Serial.printf("ERROR 0");
        }
      }

      // clear the string:
      inputString = "";
      stringComplete = false;
    }    
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
