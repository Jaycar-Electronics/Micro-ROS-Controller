#ifndef __MRC_SHARED_H__
#define __MRC_SHARED_H__
/*
Use this file as a shared resource between the two microcontrollers
*/
#define SERIAL_BAUD 115200
#define NETWORK_THROTTLE 100

#define WIFI_SSID "your wifi name"
#define WIFI_PASS "your wifi pass"
#define HOST_IP "ip.of.your.computer"
#define ROS_HOST HOST_IP ":5000/"

#endif