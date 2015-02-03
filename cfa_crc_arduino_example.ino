/*
 Crystalfontz CFA835 Serial Interfacing Example.
 
 Hardware connection:
 * Connect the serial RX pin on the Arduino to the serial
     TX pin on the CFA835 (pin 2 on the H1 connector).
 * Connect the serial TX pin on the Arduino to the serial
     RX pin on the CFA835 (pin 1 on the H1 connector).   
 * Connect GND on the Arduino to GND on the CFA835 (pin 15
     on the H1 connector, or any of the other GND points).

 See the Arduino manual for the appropriate RX & TX pins
 for your specific board.
 The example below was tested on an Arduino UNO.
 
 CFA835 Setup:
 The CFA835 serial port needs to be setup for 9600 baud.
 * Plug the CFA835 into your PC's USB port.
 * Run cfTest, and connect to the CFA835
 * Select "Interface Options" in the "Packet Type" list.
 * Enter "\000\047\002" in the Data entry box (no quotes)
     and click the "Send Packet" button. You should see
     a "33 (ack)" "NO DATA" in the Packet Flow list which
     means the CFA835 has now set the new serial port
     options.

*/

#include <SoftwareSerial.h>
#include <util/crc16.h>

//arduino serial pins
#define RX_PIN 8
#define TX_PIN 9

//////////////////////////////

typedef struct
{
  uint8_t   command;
  uint8_t   length;
  uint8_t   data[24];
  uint16_t  crc;
} CFPacket_t;

class CrystalfontzPacketLCD
{
public:
  //vars
  //functions
  CrystalfontzPacketLCD(SoftwareSerial *serialPort);
  void sendPacket(CFPacket_t *packet);
  void writeText(uint8_t x, uint8_t y, char *text, uint8_t length);
  void clearScreen(void);

private:
  //vars
  SoftwareSerial  *port;
  //functions
  uint16_t CRC(uint8_t *ptr, uint16_t len);
};


//////////////////////////////

CrystalfontzPacketLCD::CrystalfontzPacketLCD(SoftwareSerial *serialPort)
{
  //save the serial port
  port = serialPort;
}

void CrystalfontzPacketLCD::sendPacket(CFPacket_t *packet)
{
  //calculate the crc
  packet->crc = CRC((uint8_t*)packet, packet->length+2);
  
  //send the packet in sections
  port->write(packet->command);  
  port->write(packet->length);
  port->write(packet->data, packet->length);
  port->write((uint8_t*)&packet->crc, 2);
}


void CrystalfontzPacketLCD::writeText(uint8_t x, uint8_t y, char *text, uint8_t length)
{
  CFPacket_t  packet;
  
  //setup the packet
  packet.command = 31;
  packet.length = length + 2;
  packet.data[0] = x;
  packet.data[1] = y;
  memcpy(packet.data + 2, text, length);
  
  //send the packet
  sendPacket(&packet);
}

void CrystalfontzPacketLCD::clearScreen(void)
{
  CFPacket_t  packet;
  
  //setup the packet
  packet.command = 6;
  packet.length = 0;

  //send the packet
  sendPacket(&packet);
}

uint16_t CrystalfontzPacketLCD::CRC(uint8_t *data, uint16_t length)
{
  //calculate the CRC for the packet data
  uint16_t crc = 0xFFFF;
  while(length--)
    crc = _crc_ccitt_update(crc, *data++);
  return ~crc;
}

////////////////////////////////////////////

//local variables
SoftwareSerial          *softSerial;
CrystalfontzPacketLCD   *cfPacket;

void setup()
{ 
  //firmware setup
  
  //set LED pin as output
  pinMode(13, OUTPUT);
  
  //setup the serial port pins and SoftwareSerial library
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  softSerial = new SoftwareSerial(RX_PIN, TX_PIN);
  softSerial->begin(9600); //9600 baud
  
  //setup the crystalfontz packet interface (CFA533, CFA631, CFA633, CFA635, CFA735, CFA835)
  cfPacket = new CrystalfontzPacketLCD(softSerial);
  
  //init random
  randomSeed(167);
}

void loop()
{
  //the main loop
  
  //clear the LCD display
  cfPacket->clearScreen();
  //write text to a random location on the LCD display
  cfPacket->writeText(random(20-6), random(4), "CF-LCD", 6);
  
  //blink the LED
  digitalWrite(13, HIGH);  
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

