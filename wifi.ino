//#include <WebSockets.h>
#include <WiFi.h>
#include <WiFiClient.h>
 
const char* ssid     = "Pyroxicity_5g";
const char* password = "pyroxcity39";
const char* host = "192.168.0.1";


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Netmask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
}
 

 
void loop() 
{
  {
String a="HP224";
   }

  Serial.print("connecting to ");
  Serial.println(host);

  WiFiClient client;
  const int httpPort = 44444;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  client.write("HP108");
  delay(500);
  
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  
  Serial.println();
  Serial.println("closing connection");
  delay(3000);
}
