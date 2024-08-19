#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

const float pi = 3.14;
const uint16_t LASER_MSG_SIZE = 16;

rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan scan_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile float distance[LASER_MSG_SIZE]; //distance data x 16
volatile uint8_t tempData[100]; //received bytes
volatile unsigned char readDataState = 0; //reading state 0:header 1:data
volatile int startAngle = 0;
volatile int endAngle = 0;
volatile int subCount = 0; //for counting received bytes
volatile bool byteLM = false; //LSB/MSB
volatile uint8_t tempByte = 0; //now received
volatile uint8_t lastByte = 0; //previously received

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
}

void init_scan_msg() {
  scan_msg.header.frame_id.data = const_cast<char*>("laser_frame");
  scan_msg.header.stamp.sec = millis() / 1000;
  //scan_msg.angle_min
  //scan_msg.angle_max
  //scan_msg.angle_increment
  scan_msg.range_min = 0.0;
  scan_msg.range_max = 10.0;
  scan_msg.ranges.data = (float *)calloc(LASER_MSG_SIZE,sizeof(float));
  scan_msg.ranges.size = LASER_MSG_SIZE;
}

void init_lidar() {
  Serial1.begin(230400,SERIAL_8N1,13,15); //hardware serial gpio13 -> tx(lidar)
  for(int i=0;i<LASER_MSG_SIZE;i++){
    distance[i] = 0.0;
  }
}

void read_data() {
  tempByte = Serial1.read(); //read 1 byte
  if(tempByte == 0xAA && lastByte == 0x55){ //received header
    readDataState = 1;
    subCount = 0;
    byteLM = false;
  }else if(readDataState == 1){ //read data part
    tempData[subCount] = tempByte;
    if(subCount == 5){
      startAngle = ((tempData[5]<<8) + tempData[4] - 40960)/64;
    }else if (subCount == 55){
      endAngle = ((tempData[55]<<8) + tempData[54] - 40960)/64;
    }else if(subCount == 56){
      for(int i=0;i<16;i++){
        distance[i] = float(((tempData[7+3*i] & 0x3f)<<8) | tempData[6+3*i])*0.001;
      }
      // create scan message
      // need to reverse direction  lidar:clockwise ros:counterclockwise
      scan_msg.angle_max = 360. - float(startAngle) * pi / 180.;
      scan_msg.angle_min = 360. - float(endAngle) * pi / 180.;
      if(endAngle < startAngle){
        scan_msg.angle_increment = float(endAngle + 360. - startAngle) * pi /180. / LASER_MSG_SIZE;
      }else{
        scan_msg.angle_increment = float(endAngle - startAngle) * pi /180. / LASER_MSG_SIZE;
      }
      for (int i = 0; i < LASER_MSG_SIZE; i++) {
        scan_msg.ranges.data[i] = distance[LASER_MSG_SIZE+1 - i];
      }
      RCSOFTCHECK(rcl_publish(&publisher, &scan_msg, NULL));
      readDataState = 0;
    }
    subCount++;
  }
  lastByte = tempByte;
}

void setup() {
  set_microros_wifi_transports("ID", "PASSWORD", "192.168.0.8", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "topic_name"));

  init_scan_msg();
  init_lidar();
  rcl_publish(&publisher, &scan_msg, NULL);
}

void loop() {
  if(Serial1.available()){
    read_data();
  }
}
