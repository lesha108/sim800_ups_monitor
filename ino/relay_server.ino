
#include <ESP8266WiFi.h>                      // Подключаем библиотеку ESP8266WiFi
 
const char* ssid = "MY_SSID";     // Название Вашей WiFi сети
const char* password = "password";// Пароль от Вашей WiFi сети
IPAddress local_ip(192,168,1,75);      // pre-defined IP address values
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
int value = HIGH;

#define RELAY 0                               // Пин к которому подключено реле
//#define LED 1                                 // GPIO1/TXD01 - светодиод
WiFiServer server(80);                        // Указываем порт Web-сервера
 
void setup(){
  delay(2200);      
  Serial.begin(115200);                       // Скорость передачи 115200 
  pinMode(RELAY,OUTPUT);                      // Указываем вывод RELAY как выход
  //pinMode(LED,OUTPUT);                        // Указываем вывод RELAY как выход
  digitalWrite(RELAY, HIGH);                  // Устанавливаем RELAY в HIGH //LOW (0В)
  //digitalWrite(LED, HIGH);                    // Устанавливаем LED в HIGH
  Serial.println();                           // Печать пустой строки 
  Serial.print("Connecting to ");             // Печать "Подключение к:"
  Serial.println(ssid);                       // Печать "Название Вашей WiFi сети"

  WiFi.config(local_ip, gateway, subnet);
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
 
  server.begin();                             // Запуск сервера
  Serial.println("Server started");           // Печать "Server starte"
  Serial.print("Use this URL to connect: ");  // Печать "Use this URL to connect:" 
  Serial.print(WiFi.localIP());               // Печать выданого IP адресса          
}
 
void loop(){
  if (!WiFi.isConnected()) {
    ESP.restart();
  };
  
  WiFiClient client = server.available();    // Получаем данные, посылаемые клиентом 
  if (!client)                                
  {
    return;
  }
  Serial.println("new client");               // Отправка "new client"
  while(!client.available())                  // Пока есть соединение с клиентом 
  {
    delay(1);                                 // пауза 1 мс
  }

  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  if (request.indexOf("/RELAY=ON") != -1)  
  {
    Serial.println("RELAY=ON");
    digitalWrite(RELAY,LOW);
    //digitalWrite(LED,LOW);
    value = LOW;
  }
  if (request.indexOf("/RELAY=OFF") != -1)  
  {
    Serial.println("RELAY=OFF");
    digitalWrite(RELAY,HIGH);
    //digitalWrite(LED,HIGH);
    value = HIGH;
  }
  if (request.indexOf("/RELAY=RESET") != -1)  
  {
    Serial.println("RELAY=Resetting...");
    digitalWrite(RELAY,LOW);
    //digitalWrite(LED,LOW);
    delay(5000);   
    digitalWrite(RELAY,HIGH);
    //digitalWrite(LED,HIGH);
    value = HIGH;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); 
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><title>ESP8266 Router RELAY Control</title></head>");
  client.print("Relay is now: ");
 
  if(value == HIGH) 
  {
    client.print("OFF");
  } 
  else 
  {
    client.print("ON");
  }
  client.println("<br><br>");
  client.println("Turn <a href=\"/RELAY=OFF\">OFF</a> RELAY<br>");
  client.println("Turn <a href=\"/RELAY=ON\">ON</a> RELAY<br>");
  client.println("<a href=\"/RELAY=RESET\">RESET</a> RELAY<br>");
  
  if (request.indexOf("/RELAY=RESET") != -1)  
  {
    client.println("RESET OK<br></html>");
  }

  client.println("</html>");
 
  delay(1);
  Serial.println("Client disconnected");
  Serial.println("");
}
