
int EN = 2;
const int adc_pin = 0;
#include <SoftwareSerial.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>

#include <Wire.h>
#include <VL53L0X.h>
#include "AX12A.h"

#define XSHUT_pin3 7
#define XSHUT_pin2 6
#define XSHUT_pin1 5

#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;

int del = 5;

unsigned char Checksum;
unsigned char Direction_Pin;
unsigned char Time_Counter;
unsigned char Incoming_Byte;
unsigned char Position_High_Byte;
unsigned char Position_Low_Byte;
unsigned char Speed_High_Byte;
unsigned char Speed_Low_Byte;
unsigned char Load_High_Byte;
unsigned char Load_Low_Byte;

int Moving_Byte;
int RWS_Byte;
int Speed_Long_Byte;
int Load_Long_Byte;
int Position_Long_Byte;
int Temperature_Byte;
int Voltage_Byte;
int Error_Byte;

int returned_Byte;

int read_error(void);


int servo_data = 0;
int flag = 0;

void servo_cb( const std_msgs::UInt16& cmd_msg) {
  servo_data = map(cmd_msg.data, 0, 300, 0, 1023);
  flag = 1;

}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);





ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "US1", &range_msg);
ros::Publisher pub_range1( "US2", &range_msg);
ros::Publisher pub_range2( "US3", &range_msg);
ros::Publisher pub_range3( "US4", &range_msg);
ros::Publisher pub_range4( "US5", &range_msg);
ros::Publisher pub_range5( "US6", &range_msg);


ros::Publisher pub_range_( "/vl53l0x_1", &range_msg);
ros::Publisher pub_range_2("/vl53l0x_2", &range_msg);
ros::Publisher pub_range_3("/vl53l0x_3", &range_msg);
ros::Publisher pub_range_4("/vl53l0x_4", &range_msg);







const int analog_pin = 0;
unsigned long range_timer;
unsigned long range_timer2;

SoftwareSerial mySerial(10, 11);
char frameid[] = "/US1";
static unsigned long timePoint[6];
void measureDistance(byte device) {
  digitalWrite(EN, HIGH);
  uint8_t DScmd[6] = {0x55, 0xaa, device, 0x00, 0x01, 0x00};
  for (int i = 0; i < 6; i++) {
    mySerial.write(DScmd[i]);
    DScmd[5] += DScmd[i];
  }
  delay(30);
  uint8_t STcmd[6] = {0x55, 0xaa, device, 0x00, 0x02, 0x00};
  for (int i = 0; i < 6; i++) {
    mySerial.write(STcmd[i]);
    STcmd[5] += STcmd[i];
  }
  delay(5);
}

int readDistance() {
  uint8_t data[8];
  digitalWrite(EN, LOW);
  boolean done = false;
  int counter = 0;
  int result = -1;

  while (!done) {
    int bytes = mySerial.available();
    if (bytes == 8) {
      for (int i = 0; i < 8; i++) {
        data[i] = mySerial.read();
      }
      result = (int)data[5] * 256 + data[6];
      done = true;
    } else {
      delay(del);
      counter++;
      if (counter == 5) {
        done = true;
      }
    }
  }
  return result;
}

void setup() {


  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);

  Wire.begin();

  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);

  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();


  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);
  Sensor4.setTimeout(500);

  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor3.startContinuous();
  Sensor4.startContinuous();


  nh.initNode();

  nh.subscribe(sub);

  //  mySerial.begin(1000000);
  Serial2.begin(1000000);
  setEndless(254, OFF);

  nh.advertise(pub_range);
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  nh.advertise(pub_range5);


  nh.advertise(pub_range_);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  nh.advertise(pub_range_4);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.04;
  range_msg.max_range = 5.0;

  nh.spinOnce();
  pinMode(EN, OUTPUT);
  mySerial.begin(19200);
  delay(200);
  nh.spinOnce();
  digitalWrite(EN, HIGH);
  delay(2000);
  nh.spinOnce();
}






long range_time;

void loop()
{

  //servo
  if (flag == 1) {
    moveSpeed(254, servo_data, 100);
  }

  nh.spinOnce();
  //  sonars
  if ( millis() >= range_time ) {
    int r = 0;
    for (int id = 0; id < 6; id++) {
      measureDistance(0x11 + id);
      range_msg.range = readDistance() / 100.00;
      range_msg.header.stamp = nh.now();
      switch (id) {
        case 0:
          pub_range.publish(&range_msg);
          break;
        case 1:
          pub_range1.publish(&range_msg);
          break;
        case 2:
          pub_range2.publish(&range_msg);
          break;
        case 3:
          pub_range3.publish(&range_msg);
          break;
        case 4:
          pub_range4.publish(&range_msg);
          break;
        case 5:
          pub_range5.publish(&range_msg);
          break;
      }
    }
    range_time =  millis() + 25;
  }

  //vl53
  if ( (millis() - range_timer2) > 50) {

    range_msg.range = Sensor4.readRangeContinuousMillimeters() / 1000.00;
    range_msg.header.stamp = nh.now();
    pub_range_.publish(&range_msg);

    range_msg.range = Sensor1.readRangeContinuousMillimeters() / 1000.00;
    range_msg.header.stamp = nh.now();
    pub_range_2.publish(&range_msg);

    range_msg.range = Sensor2.readRangeContinuousMillimeters() / 1000.00;
    range_msg.header.stamp = nh.now();
    pub_range_3.publish(&range_msg);

    range_msg.range = Sensor3.readRangeContinuousMillimeters() / 1000.00;
    range_msg.header.stamp = nh.now();
    pub_range_4.publish(&range_msg);

    range_timer2 =  millis();
  }
}



int move(unsigned char ID, int Position)
{
  char Position_H, Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;

  const unsigned int length = 9;
  unsigned char packet[length];

  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

  packet[0] = AX_START;
  packet[1] = AX_START;
  packet[2] = ID;
  packet[3] = AX_GOAL_LENGTH;
  packet[4] = AX_WRITE_DATA;
  packet[5] = AX_GOAL_POSITION_L;
  packet[6] = Position_L;
  packet[7] = Position_H;
  packet[8] = Checksum;

  return (sendPacket(packet, length));
}

int sendPacket(unsigned char *packet, unsigned int length) {
  for (int i = 0; i < length; i++) {
    Serial2.write( packet[i]);
  }
}

//void begin(long baud, unsigned char directionPin, HardwareSerial *srl)
//{
//  varSerial = srl;
//  Direction_Pin = directionPin;
//  setDPin(Direction_Pin, OUTPUT);
//  beginCom(baud);
//}


int setEndless(unsigned char ID, bool Status)
{
  if ( Status )
  {
    const unsigned int length = 9;
    unsigned char packet[length];

    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_CCW_ANGLE_LIMIT_L;
    packet[6] = 0;            // full rotation
    packet[7] = 0;            // full rotation
    packet[8] = Checksum;

    return (sendPacket(packet, length));
  }
  else
  {
    //    turn(ID,0,0);

    const unsigned int length = 9;
    unsigned char packet[length];

    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_CCW_ANGLE_LIMIT_L;
    packet[6] = AX_CCW_AL_L;
    packet[7] = AX_CCW_AL_H;
    packet[8] = Checksum;

    return (sendPacket(packet, length));
  }
}

int moveSpeed(unsigned char ID, int Position, int Speed)
{
  char Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position;                // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

  const unsigned int length = 11;
  unsigned char packet[length];

  Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

  packet[0] = AX_START;
  packet[1] = AX_START;
  packet[2] = ID;
  packet[3] = AX_GOAL_SP_LENGTH;
  packet[4] = AX_WRITE_DATA;
  packet[5] = AX_GOAL_POSITION_L;
  packet[6] = Position_L;
  packet[7] = Position_H;
  packet[8] = Speed_L;
  packet[9] = Speed_H;
  packet[10] = Checksum;

  return (sendPacket(packet, length));
}
