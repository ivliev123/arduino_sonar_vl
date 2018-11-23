/* 
 * rosserial IR Ranger Example  
 * 
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>


#include <Wire.h>
#include <VL53L0X.h>

//#define XSHUT_pin6 not required for address change
//#define XSHUT_pin5 9
//#define XSHUT_pin4 8
#define XSHUT_pin3 7
#define XSHUT_pin2 6
#define XSHUT_pin1 5

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
//#define Sensor5_newAddress 45
//#define Sensor6_newAddress 46

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;



ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/vl53l0x_1", &range_msg);
ros::Publisher pub_range2("/vl53l0x_2", &range_msg);
ros::Publisher pub_range3( "/vl53l0x_3", &range_msg);
ros::Publisher pub_range4("/vl53l0x_4", &range_msg);

const int analog_pin = 0;
unsigned long range_timer;

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 * 
 * NOTE: This function is only applicable to the GP2D120XJ00F !!
 * Using this function with other Rangers will provide incorrect readings.
 */
float getRange(int pin_num){
    int sample;
    // Get data
    sample = analogRead(pin_num)/4;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(sample < 10)
        return 254;     // max range
    // Magic numbers to get cm
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}

char frameid[] = "/US1";

void setup()
{

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
  nh.advertise(pub_range);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    
    range_msg.range = Sensor4.readRangeContinuousMillimeters()/1000.00;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);

    range_msg.range = Sensor1.readRangeContinuousMillimeters()/1000.00;
    range_msg.header.stamp = nh.now();
    pub_range2.publish(&range_msg);

    range_msg.range = Sensor2.readRangeContinuousMillimeters()/1000.00;
    range_msg.header.stamp = nh.now();
    pub_range3.publish(&range_msg);

    range_msg.range = Sensor3.readRangeContinuousMillimeters()/1000.00;
    range_msg.header.stamp = nh.now();
    pub_range4.publish(&range_msg);
    
    range_timer =  millis();
  }
  nh.spinOnce();
}
