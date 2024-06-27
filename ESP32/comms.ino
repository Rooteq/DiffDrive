#include <arduino.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <bitset>

#define START_BYTE 0xAA // Znacznik początku ramki
#define END_BYTE 0x55   // Znacznik końca ramki
#define DATA_LENGTH 10   // Długość danych w ramce (3 * 2 bajty)
#define TX_MESSAGE_LENGTH 9   // Długość danych w ramce (3 * 2 bajty)
#define CRC_POLYNOMIAL 0x1021 // Standard CRC-16-ANSI polynomial

#define TX_DESTINATION_LENGTH 8

HardwareSerial uart(2);

websockets::WebsocketsClient socket;

const char* websocketServer = "ws://192.168.0.198:9999/";
bool connected = false;
// const char* ssid = "UPC7516459";
// const char* password = "v7srezmAuxwrzrwy";
const char* ssid = "Garnuch";
const char* password = "88888888";

struct Pos
{
  uint8_t flag;
  int16_t x;
  int16_t y;
  int16_t ang;
  uint8_t obstacleProximity;
};

struct Dest
{
  int16_t x;
  int16_t y;
};

void rxPos();
void rxCommand();
void txPos();
void txCommand();
uint16_t crc16(const uint8_t* data_p, uint8_t length);

void connectWifi();
void connectToWebsocket();
void handleWebSocket();
void handleMessage(websockets::WebsocketsMessage message);
void handleEvent(websockets::WebsocketsEvent event, websockets::WSInterfaceString data);

Pos pos;
Dest dest;

uint8_t flag;
uint8_t buffer[DATA_LENGTH];
char messageBuffer[TX_MESSAGE_LENGTH];

unsigned long lastSend = millis();

// ---------- MAIN FUNCTIONS -------------

void setup() {
  uart.begin (115200, SERIAL_8N1, 16, 17);
  Serial.begin(9600);

  connectWifi();
  connectToWebsocket();

  socket.onMessage(handleMessage);
  socket.onEvent(handleEvent);
}

void loop() 
{
  handleWebSocket();
  rxPos();
}


// --------- FUNCTION DEFINITIONS ------------

void rxPos()
{
  while(uart.available())
  {
    if (uart.read() == START_BYTE) {

      uart.readBytes(buffer, DATA_LENGTH);

      if(uart.read() == END_BYTE)
      {
        if (crc16(buffer, 10) == 0) {
          pos.flag = buffer[0];
          pos.x = (int16_t)(buffer[1] <<8 | (buffer[2]));
          pos.y = (int16_t)(buffer[3] << 8 | (buffer[4]));
          pos.ang = (int16_t)(buffer[5] << 8 | (buffer[6]));
          pos.obstacleProximity = buffer[7];

          txPos(); // <---------------------------- TX HERE!!!!!!!
        }
        else{
          Serial.println("Bad crc!");
          break;
        }
      }
      else
      {
        break;
      }
    }
  }
}

void txPos()
{
  messageBuffer[0] = 'P';
  messageBuffer[1] = pos.flag;

  messageBuffer[3] = (uint8_t)(pos.x & 0xFF);
  messageBuffer[2] = (uint8_t)((pos.x >> 8) & 0xFF);
  messageBuffer[5] = (uint8_t)(pos.y & 0xFF);
  messageBuffer[4] = (uint8_t)((pos.y >> 8) & 0xFF);
  messageBuffer[7] = (uint8_t)(pos.ang & 0xFF);
  messageBuffer[6] = (uint8_t)((pos.ang >> 8) & 0xFF);
  messageBuffer[8] = pos.obstacleProximity;

  socket.sendBinary(messageBuffer, TX_MESSAGE_LENGTH);

  // Serial.println("----BOT:----");
  //Serial.println(pos.flag);
  // Serial.println(pos.x);
  // Serial.println(pos.y);
  // Serial.println(pos.ang);
  // Serial.println("----EOT----\n");
}

void rxCommand()
{

}

void txCommand()
{

}

void sendDestination()
{
  uint8_t message[TX_DESTINATION_LENGTH];
  message[0] = START_BYTE;

  message[2] = (uint8_t)(dest.x & 0xFF);
  message[1] = (uint8_t)((dest.x >> 8) & 0xFF);
  message[4] = (uint8_t)(dest.y & 0xFF);
  message[3] = (uint8_t)((dest.y >> 8) & 0xFF);


  int16_t crc = crc16(&message[1], 4);

  message[6] = (uint8_t)(crc & 0xFF);
  message[5] = (uint8_t)((crc >> 8) & 0xFF);

  message[7] = END_BYTE;

  uart.write(message, TX_DESTINATION_LENGTH);
  Serial.println("Sent destination");
}

void handleMessage(websockets::WebsocketsMessage message)
{
  if(message.data()[0] == 'D')
  {
    dest.x = (int16_t)((message.rawData()[1] << 8) | message.rawData()[2]);
    dest.y = (int16_t)((message.rawData()[3] << 8) | message.rawData()[4]);

    sendDestination();

    return;
  }

  String data = message.data();
  if(data == "MS")
  {
      const char* mess = "MS";
      uart.write(mess);
  }
  else if(data == "MF")
  {
      const char* mess = "MF";
      uart.write(mess);
  }
  else if(data == "MB")
  {
      const char* mess = "MB";
      uart.write(mess);
  }
  else if(data == "ML")
  {
      const char* mess = "ML";
      uart.write(mess);
  }
  else if(data == "MR")
  {
      const char* mess = "MR";
      uart.write(mess);
  }
  else
  {
    Serial.println("Wrong message!");
  }
  Serial.println(data);
}

void handleEvent(websockets::WebsocketsEvent event, websockets::WSInterfaceString data)
{
  // TODO: implement
}

void handleWebSocket()
{
  if(!connected)
  {
    Serial.println("Connecting to WebSocket server");
    connectToWebsocket();
  }

  socket.poll();
}

void connectToWebsocket(){
  connected = socket.connect(websocketServer);
  if(connected)
  {
    Serial.println("Connected to websocket");
  }
  else
  {
    Serial.println("Failed to connect to websocket!");
  }
}

void connectWifi(){
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);

  Serial.println("Connecting to WiFi");

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected to: " + String(ssid));
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
}


uint16_t crc16(const uint8_t* data_p, uint8_t length){
    uint8_t x;
    uint16_t crc = 0x1D0F;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

