#ifndef DEFINE_H
#define DEFINE_H

#define     CAN_DEFAULT_PORT_NAME           "can0"

//        ________________________
//        ::: DEFINITION TOPIC :::
#define     TOPIC_IMU                       "ibot_can_imu"
#define     TOPIC_ALARME_BATTERIE           "ibot_can_battery_alarm"
#define     TOPIC_CMD_MOTOR                 "secure_twist"

//        __________________________
//        ::: DEFINITION SERVICE :::
#define     SERVICE_US_LEFT                 "ibot_us_left"
#define     SERVICE_US_FRONT                "ibot_us_front"
#define     SERVICE_US_GROUND               "ibot_us_ground"
#define     SERVICE_US_RIGHT                "ibot_us_right"

#define     SERVICE_POWER_SUPPLY            "ibot_power_supply"


//        _________________________
//        ::: DEFINITION CAN ID :::

//    > Motor Left :
#define     CAN_ID_MOTOR_LEFT_SPEED_ORDER   0x30
#define     CAN_ID_MOTOR_LEFT_CONFIG        0x31
#define     CAN_ID_MOTOR_LEFT_PID           0x32
#define     CAN_ID_MOTOR_LEFT_SPEED_REAL    0x33

//    > Motor Right :
#define     CAN_ID_MOTOR_RIGHT_SPEED_ORDER  0x35
#define     CAN_ID_MOTOR_RIGHT_CONFIG       0x36
#define     CAN_ID_MOTOR_RIGHT_PID          0x37
#define     CAN_ID_MOTOR_RIGHT_SPEED_REAL   0x38

//    > Supply Board :
#define     CAN_ID_BATTERY_ALARM            0x80

//    > Inertial Unit Sensor :
#define     CAN_ID_IMU_DATA                 0x90
#define     CAN_ID_IMU_TEST                 0x190

//    > Power Supply Board :
#define     CAN_ID_POWER_SUPPLY             0x91

//    > Ultrasonic sensors :
#define     CAN_ID_US_RIGHT                 0xC0
#define     CAN_ID_US_FRONT                 0xC1
#define     CAN_ID_US_GROUND                0xC2
#define     CAN_ID_US_LEFT                  0xC3


//        ________________________________________________
//        ::: CONVERSION DIGITAL VALUE TO ANALOG VALUE :::
#define     CONVERSION_VBAT_TO_5V           0.01637f
#define     CONVERSION_12V_TO_5V            0.011728f
#define     CONVERSION_5V_TO_5V             0.004887f
#define     COEF_IBAT_TO_5V                 0.0391
#define     OFFSET_IBAT_TO_5V               20.0f


#endif // DEFINE_H
