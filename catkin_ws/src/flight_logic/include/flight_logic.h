typedef enum {
    INIT               ,
    TAKEOFF            ,
    STAND_BY           ,
    LANDING            ,
    RELEASE_CONTROL    ,
    SEARCH_FOR_TAGS    ,
    UGV_TRACKING       ,


} MISSION_STATUS;

struct Triple {

    float vec[3];

};

struct Tuple {

    float vec[2];

};

void printMap(int waypointIndex);

/*
//TODO Add 2 Watchdogs for timing
Publisher target_pos_pub;     // Publish the target position to PID controller
Publisher cmd_msg_pub;        // Publish the control message (obtain control, takeoff, landing, etc.) to API
Publisher pid_param_pub;      // Publish the updated PID parameters set to PID controller
Publisher pid_ctrl_limit_pub; // Publish the updated velocity limit to PID controller 
Publisher stm32_cmd_pub;      // Publish the commands to STM32 transceiver node
Publisher ugv_activation_pub; // Publish the activation message to UGV
Publisher tracking_pub;

Subscriber ctrl_vel_sub;  // Subscribe the desired velocity from PID controller
Subscriber pos_error_sub; // Subscribe the flag for arrival at target position
Subscriber ugv_pos_sub; // Subscribe the current position of UGV from ekf node
Subscriber current_pos_sub; // Subscribe the current position from pid controller (originally from ekf and Guidance ultrasonic)
Subscriber detected_marker_sub; // Subscribe the detected markers and record their locations in local frame
*/
