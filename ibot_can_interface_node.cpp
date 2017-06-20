#include <ros/ros.h>

#include "src/IT_CAN.h"

#include "src/parser/CAN2ROS/ParserCan2Ros4IMU.h"
#include "src/parser/CAN2ROS/ParserCan2RosAlarmeBatterie.h"

#include "src/parser/ROS2CAN/ParserRos2Can4RequestSpeed.h"

#include "src/parser/CAN2SERVICEROS/ParserCan2ServiceRos4Ultrason.h"
#include "src/parser/CAN2SERVICEROS/ParserCan2ServiceRos4PowerSupply.h"

#include "src/define.h"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ibot_can_interface_node");
  ros::NodeHandle node;

  //  CAN Interface :

  /// Object representing the CAN interface :
  IT_CAN*     can_interface = new IT_CAN();

  //  __________________
  //  ::: Def PARSER :::

  ROS_INFO("HERE 1");
  ///       ##############################
  ///       --- DATA FRAMES CAN TO ROS ---
  ///       ##############################

  /**
    For adding CAN frame Parser, you must create an object child of abstract parser and give it to the can interface :
    **/



  //  > IMU :
  int  can_imu_id;
  std::string can_imu_topic;
  if(!node.getParam("can_imu_id",(int&)can_imu_id))
    can_imu_id = CAN_ID_IMU_DATA;
  if(!node.getParam("can_imu_topic",can_imu_topic))
    can_imu_topic = TOPIC_IMU;

  ParserCan2Ros4IMU *parser_imu = new ParserCan2Ros4IMU(node,"ibot_can_imu",100,can_interface,10);
  can_interface->add_parser(can_imu_id,parser_imu);

  ROS_INFO("HERE 3");
  //  > ALARME BATTERIE :
  int can_alarm_batt_id;
  std::string can_alarm_batt_topic;
  if(!node.getParam("can_battery_alarm_id",(int&)can_alarm_batt_id))
    can_alarm_batt_id = CAN_ID_BATTERY_ALARM;
  if(!node.getParam("can_battery_alarm_topic",can_alarm_batt_topic))
    can_alarm_batt_topic = TOPIC_ALARME_BATTERIE;
  ParserCan2RosAlarmeBatterie *parser_battery_alarm = new ParserCan2RosAlarmeBatterie(node,can_alarm_batt_topic.c_str(),10,CONVERSION_VBAT_TO_5V,new Alarme_Batterie_Config());
  can_interface->add_parser(can_alarm_batt_id,parser_battery_alarm);

  ROS_INFO("HERE 4");
  ///        ##################################
  ///        --- DATA FRAME FROM ROS TO CAN ---
  ///        ##################################

  std::string topic_left_cmd_vel, topic_right_cmd_vel;
  int can_motor_left_id, can_motor_right_id;


  if(!node.getParam("can_motor_left_topic",topic_left_cmd_vel))
    topic_left_cmd_vel = TOPIC_CMD_MOTOR;

  ROS_INFO("HERE 5");
  if(!node.getParam("can_motor_right_topic",topic_right_cmd_vel))
    topic_right_cmd_vel = TOPIC_CMD_MOTOR;

  ROS_INFO("HERE 6");
  if(!node.getParam("can_motor_left_id",(int&)can_motor_left_id))
    can_motor_left_id = CAN_ID_MOTOR_LEFT_SPEED_ORDER;

  ROS_INFO("HERE 7");
  if(!node.getParam("can_motor_right_id",(int&)can_motor_right_id))
    can_motor_right_id = CAN_ID_MOTOR_RIGHT_SPEED_ORDER;

  ROS_INFO("HERE 8");
  //  > LEFT MOTOR FRAMES :
  //ParserRos2Can4RequestSpeed  *parser_motor_left_speed_order = new ParserRos2Can4RequestSpeed(can_interface,CAN_ID_MOTOR_LEFT_SPEED_ORDER,node,"ibot_can_motor_left_speed_order",10,false);
  ParserRos2Can4RequestSpeed  *parser_motor_left_speed_order = new ParserRos2Can4RequestSpeed(can_interface,can_motor_left_id,node,topic_left_cmd_vel.c_str(),10,false);

  ROS_INFO("HERE 9");
  //  > RIGHT MOTOR FRAMES :
  //ParserRos2Can4RequestSpeed  *parser_motor_right_speed_order = new ParserRos2Can4RequestSpeed(can_interface,CAN_ID_MOTOR_RIGHT_SPEED_ORDER,node,"ibot_can_motor_right_speed_order",10,false);
  ParserRos2Can4RequestSpeed  *parser_motor_right_speed_order = new ParserRos2Can4RequestSpeed(can_interface,can_motor_right_id,node,topic_right_cmd_vel.c_str(),10,true);

  ROS_INFO("HERE 10");
  ///        ###################################
  ///        --- RTR CAN FRAME = ROS SERVICE ---
  ///        ###################################
  int can_us_id;
  std::string service_us;


  //  > ULTRASONIC SENSORS :

  if(!node.getParam("can_us_left_id",(int&)can_us_id))
    can_us_id = CAN_ID_US_LEFT;
  if(!node.getParam("can_us_left_service",service_us))
    service_us = SERVICE_US_LEFT;
  ParserCan2ServiceRos4Ultrason*  parser_us_left = new ParserCan2ServiceRos4Ultrason(can_interface,node,service_us.c_str(),can_us_id,can_us_id);
  can_interface->add_parser(can_us_id,parser_us_left);

  ROS_INFO("HERE 11");
  if(!node.getParam("can_us_front_id",(int&)can_us_id))
    can_us_id = CAN_ID_US_FRONT;
  if(!node.getParam("can_us_front_service",service_us))
    service_us = SERVICE_US_FRONT;
  ParserCan2ServiceRos4Ultrason*  parser_us_front = new ParserCan2ServiceRos4Ultrason(can_interface,node,service_us.c_str(),can_us_id,can_us_id);
  can_interface->add_parser(can_us_id,parser_us_front);

  ROS_INFO("HERE 12");
  if(!node.getParam("can_us_ground_id",(int&)can_us_id))
    can_us_id = CAN_ID_US_GROUND;
  if(!node.getParam("can_us_ground_service",service_us))
    service_us = SERVICE_US_GROUND;
  ParserCan2ServiceRos4Ultrason*  parser_us_ground = new ParserCan2ServiceRos4Ultrason(can_interface,node,service_us.c_str(),can_us_id,can_us_id);
  can_interface->add_parser(can_us_id,parser_us_ground);

  ROS_INFO("HERE 13");
  if(!node.getParam("can_us_right_id",(int&)can_us_id))
    can_us_id = CAN_ID_US_RIGHT;
  if(!node.getParam("can_us_right_service",service_us))
    service_us = SERVICE_US_RIGHT;

  ParserCan2ServiceRos4Ultrason*  parser_us_right = new ParserCan2ServiceRos4Ultrason(can_interface,node,service_us.c_str(),can_us_id,can_us_id);
  can_interface->add_parser(can_us_id,parser_us_right);

  ROS_INFO("HERE 14");
  //  > POWER SUPPLY :
  int can_power_supply_id;
  std::string service_power_supply;
  if(!node.getParam("can_power_supply_id",(int&)can_power_supply_id))
    can_power_supply_id = CAN_ID_POWER_SUPPLY;
  if(!node.getParam("can_power_supply_service",service_power_supply))
    service_power_supply = SERVICE_POWER_SUPPLY;
  ParserCan2ServiceRos4PowerSupply* parser_powersupply = new ParserCan2ServiceRos4PowerSupply(can_interface,node,service_power_supply.c_str(),can_power_supply_id,can_power_supply_id);
  can_interface->add_parser(can_power_supply_id,parser_powersupply);

  ROS_INFO("HERE 15");
  // Start the CAN Interface internal thread for recv function :
  can_interface->start();


  ///    ############
  ///    --- MAIN ---
  ///    ############
  //  Main loop is managed by ROS (callback from ROS)
  ros::spin();
  // After that, callback canot be called

  // Stop the CAN Interface internal thread for recv function :
  can_interface->stop();

  // delete the CAN Interface :
  delete can_interface;
  delete parser_imu;
  delete parser_battery_alarm;

  delete parser_motor_left_speed_order;
  delete parser_motor_right_speed_order;

  delete parser_us_left;
  delete parser_us_ground;
  delete parser_us_front;
  delete parser_us_right;
  delete parser_powersupply;

  return EXIT_FAILURE;
}
