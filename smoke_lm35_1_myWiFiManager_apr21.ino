/*
   smoke sensor kit UDP communication



*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <stdlib.h>
#include <string.h>
#include <PubSubClient.h>
#include <SimpleDHT.h>

#define TRIGGER_PIN     12        //  GPIO12 , D6
#define HEART_BEAT_PIN  13        // GPIO13  , D7
#define DHT22_PIN       0         // GPIO0    ,D3
#define RATE_ADC_CENTIGRADE 3.29 //adc/centrgrade = 3.29adc/centigrade
#define SMOKE_DIGI_PIN  5       //D2 pin on nodemcu

//const char* mqtt_server = "broker.hivemq.com";
WiFiServer server(80);

char* ssid = "abcabc";
char* password = "123456789";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[255];
char replyPacket[] = "Hi there! Got the message :-)";

long timer1 = 15 * 60 * 1000; // 3 minutes
long timer1StartTime = 0;
bool isTimer1_on = false;
bool state = false;
int temperatureCutOff = (int)90 * RATE_ADC_CENTIGRADE ; //in ADC value since 3.29 * temperature = ADC.  i.e 3.29 adc = 1 C = 10mV
int temperature_PV;   //in ADC value
float gasValue;

WiFiClient espClient;
PubSubClient client(espClient);
SimpleDHT22 dht22;
long lastMsg = 0;
char msg[75];
int value = 0;
uint8_t rcTries = 0;
uint8_t Relay1_pin = 16;
int pinDHT22 = 0;

String sysmode = "startMode";
bool shouldDcApMode = false;
long lastTime;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char message[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message[i] = payload[i];
  }
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if (strcmp(topic, "orgo/temp/1/sw1") == 0) {
    if ((char)payload[0] == '1') {
      turnOnMotor();
    } else {
      turnOffMotor();
    }
  } else if (strcmp(topic, "orgo/temp/1/timer1") == 0) {
    int valx = atoi(message);
    Serial.printf("int value received is: %d\n", valx);
    //uint16_t spx = valx;
    timer1 = valx * 60 * 1000; //milliseconds
    Serial.printf("new Timer1 :%d:", timer1);
  } else if (strcmp(topic, "orgo/temp/1/temp_cut") == 0) {
    int valx = atoi(message);
    Serial.printf("int value received is: %d\n", valx);
    //uint16_t spx = valx;
    temperatureCutOff = (int)valx * RATE_ADC_CENTIGRADE ; //Centrigrades converted to ADC (0-1024)
    Serial.printf("Cutt off temperature :%d:", temperatureCutOff);
  }
}
void turnOnMotor() {  //motor is on sw1
  if (temperature_PV  < temperatureCutOff) {
    digitalWrite(Relay1_pin, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    Serial.println("Relay1_pin High");
    timer1StartTime = millis();
    isTimer1_on = true;
  } else {
    Serial.println("Motor too hot cannot start");
    if (client.connected()) {
      client.publish("orgo/temp/1/outTopic", "Motor too hot cannot start");
    }
  }
}
void turnOffMotor() {
  digitalWrite(Relay1_pin, LOW);  // Turn the LED off by making the voltage HIGH
  Serial.println("Relay1_pin Low");
  isTimer1_on = false;
  timer1StartTime = 0;
}
char *ftoa(char *a, double f, int precision)
{
  long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};

  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

void readDHT22() {
  // read without samples.
  // @remark We use read2 to get a float data, such as 10.1*C
  //    if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
  float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err); delay(2000);
    return;
  }

  // Serial.print("Sample OK: ");
  Serial.print((float)temperature); Serial.print(" *C, ");
  Serial.print((float)humidity); Serial.println(" RH%");
  // snprintf (msg, 75, "Temperature # %d", (int)temperature);
  ftoa(msg, (double)temperature, 2);
  // Serial.println(msg);
  client.publish("temp", msg);
  ftoa(msg, (double)humidity, 2);
  //  Serial.println(msg);
  client.publish("hum", msg);
  Serial.println("published");
  // DHT22 sampling rate is 0.5HZ.
  delay(2500);
}
void readTemperatureSensor() {
  temperature_PV =  analogRead(A0);
  Serial.print("adc value:");
  Serial.println(temperature_PV);
  Serial.println(temperature_PV / RATE_ADC_CENTIGRADE); // print in Centigrade
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Orgo-2fs7")) {
      Serial.println("connected");
      rcTries = 0;
      // Once connected, publish an announcement...
      client.publish("orgo/temp/1/outTopic", "online");
      // ... and resubscribe
      client.subscribe("orgo/temp/1/temp_cut");
      client.subscribe("orgo/temp/1/sw1");
      client.subscribe("orgo/temp/1/timer1");
    } else {
      rcTries++;
      if (rcTries > 15) {
        ESP.restart();
      }
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void startAP() {
  Serial.println();
  Serial.println("Timeout!!....could not connect to WiFi....Now starting AP");
  boolean result = WiFi.softAP("orgoAPDefault", "12345678");
  if (result == true)
  {
    sysmode = "APMode";
    Serial.println("Access Point Started");
    Serial.printf("Stations connected = %d\n", WiFi.softAPgetStationNum());
    server.begin();
    Serial.printf("Web server started, open %s in a web browser\n", WiFi.softAPIP().toString().c_str());
    delay(5000);
  }
  else
  {
    Serial.println("AP Failed!");
  }
}

void setup(void)
{
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT);      // for calling on demand configuration portal
  pinMode(SMOKE_DIGI_PIN, INPUT);
  pinMode(HEART_BEAT_PIN, OUTPUT);  // heart beat
  pinMode(Relay1_pin, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(A0, INPUT);
  digitalWrite(Relay1_pin, LOW);
  digitalWrite(SMOKE_DIGI_PIN, LOW);
  digitalWrite(HEART_BEAT_PIN, HIGH);
  // delay(2*60*1000) ;  // wait for two minutes to get internet routed to be started
  Serial.println();
  //Serial.printf("Connecting to %s\n", ssid);
  Serial.printf("Connecting to previously save SSID\n");
  //WiFi.begin(ssid, password);
  WiFi.begin();
  //WiFi.config(staticIP, gateway, subnet);
  long tryTime = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - tryTime >= 1 * 15 * 1000) {
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    sysmode = "StationMode";
    Serial.println();
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
  } else {  // could not connect in station mode so start AP
    startAP();
  }
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  //  client.setServer(mqtt_server, 1883);
  //  client.setCallback(callback);
}

// prepare a web page to be send to a client (web browser)
String prepareHtmlPage()
{
  String htmlPage =
    String("HTTP/1.1 200 OK\r\n") +
    "Content-Type: text/html\r\n" +
    "Connection: close\r\n" +  // the connection will be closed after completion of the response
    "Refresh: 600\r\n" +  // refresh the page automatically every 5 sec
    "\r\n" +
    "<!DOCTYPE HTML>" +
    "<h2>Put Your Internet information</h2>"  +
    "<form action=\"/config\">  \
      SSID:<br> \
      <input type=\"text\" name=\"ssid\" value=\"\"> \
      <br>  \
      Password:<br>  \
      <input type=\"text\" name=\"lastname\" value=\"\"> \
      <br><br>  \
      <input type=\"submit\" value=\"Submit\">  \
      </form>" +
    "\r\n";
  return htmlPage;
}

String mySSID = "";
String myPSD = "";

void setWebServer() {
  WiFiClient client = server.available();
  // wait for a client (web browser) to connect
  if (client)
  {
    Serial.println("\n[Client connected]");
    while (client.connected())
    {
      // read line by line what the client (web browser) is requesting
      if (client.available())
      {
        String line = client.readStringUntil('\r');
        Serial.print(line);
        // wait for end of client's request, that is marked with an empty line
        mySSID = "";  myPSD = "";
        if (line.length() > 1 && line[0] == 'G' && line[1] == 'E' && line[2] == 'T' && line[11] == '?') {
          int firstIndexOfEquals = line.indexOf('=');
          int firstIndexOfAnd = line.indexOf('&');
          Serial.println();
          Serial.print("ssid=");

          for (int x = firstIndexOfEquals + 1 ; x <= firstIndexOfAnd - 1 ; x++) {
            Serial.print(line[x]);
            mySSID.concat(line[x]);
          }
          Serial.printf("\nssid=%s", mySSID.c_str());
          ssid = (char *)mySSID.c_str();
          Serial.println();
          int lastIndexOfEquals = line.lastIndexOf('=');
          int lastIndexOfSpace = line.lastIndexOf(' ');
          Serial.print("password=");
          for (int y = lastIndexOfEquals + 1; y <= lastIndexOfSpace - 1; y++) {
            Serial.print(line[y]);
            myPSD.concat(line[y]);
          }
          Serial.printf("\npassword=%s", myPSD.c_str());
          password = (char *)myPSD.c_str();
          Serial.println("Variables status");
          Serial.println(mySSID.equals(""));
          Serial.println(myPSD.equals(""));
          //  WiFi.softAPdisconnect(true);
          if (mySSID.equals("") == 0 && myPSD.equals("") == 0) {
            WiFi.begin(mySSID.c_str(), myPSD.c_str());
            while (WiFi.status() != WL_CONNECTED)
            {
              delay(500);
              Serial.print(".");
            }
            sysmode = "StationMode";
            Serial.println();
            Serial.print("Connected, IP address: ");
            Serial.println(WiFi.localIP());
            Serial.println("disconnecting AP");
            // WiFi.softAPdisconnect(true);    causes exception so commented
            shouldDcApMode = true;
          }

        }
        //Serial.println();

        if (line.length() == 1 && line[0] == '\n')
        {
          client.println(prepareHtmlPage());
          break;
        }
      }
    }
    delay(1); // give the web browser time to receive the data

    // close the connection:
    client.stop();
    Serial.println("[Client disonnected]");
  }
}

void loop() {
  if (sysmode.equals("StationMode")) {   // station mode
    if (shouldDcApMode == true) {
      shouldDcApMode = false;
      WiFi.softAPdisconnect(true);
    }
    lastTime = millis();
    gasValue = analogRead(A0);
    Serial.println((int)gasValue);
    int smokeDet = digitalRead(SMOKE_DIGI_PIN);
    //Serial.printf("\nsmokeDet is %d",smokeDet);
    Serial.println(smokeDet);
    snprintf (msg, 75, "%d,%d", ((int)smokeDet==1?0:1),(int)gasValue);
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);

      // send back a reply, to the IP address and port we got the packet from
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(msg);
      Udp.endPacket();
    }
    if (smokeDet != 1) {
      Serial.println("Smoke Alarm");
      Serial.println(msg);
      Udp.beginPacket("192.168.100.6", 50565);
      Udp.write(msg);
      Udp.endPacket();
    }
    delay(2000);
    digitalWrite(HEART_BEAT_PIN, (state ? HIGH : LOW));
    state = !state;
    if (digitalRead(TRIGGER_PIN) == HIGH && (1 == 2)) {
      Serial.println("Request for WIFI Reset");
      digitalWrite(HEART_BEAT_PIN, HIGH);
      // sysmode = "APMode";
      WiFi.disconnect(true); // forget ssid and password
      delay(10000);
      ESP.restart();
    }
  } else if (sysmode.equals("APMode")) { // AP mode
    // Serial.println("Wifi Not Connected");
    // delay(10000);
    setWebServer();
  }
}
