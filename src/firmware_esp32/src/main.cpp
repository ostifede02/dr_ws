#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "board_pinout.h"
#include "motor_driver.h"
#include "configuration.h"
#include "error_log.h"
#include "callback_functions.h"
#include "custom_messages.h"
#include "rcl_publishers.h"



void setup()
{
	// configure wifi transport
	// IPAddress agent_ip(192, 168, 113, 175);
	// size_t agent_port = 8888;
	// char ssid[] = "Laptop di Fede";
	// char psk[] = "12345678";
	// set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

	// configure serial transport
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(500);


    // initialize shift register pinout
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_CLOCK_PIN, OUTPUT);
    pinMode(I2S_LATCH_PIN, OUTPUT);

    // initialize limit switches pinout
    pinMode(LIMIT_SWITCH_1_PIN, INPUT);
    pinMode(LIMIT_SWITCH_2_PIN, INPUT);
    pinMode(LIMIT_SWITCH_3_PIN, INPUT);


	// micro-ROS setup
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;


	// create init_options
	RCCHECK(rclc_support_init(
		&support, 
		0, 
		NULL, 
		&allocator));


	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(
		&node, 
		"motor_driver_node", 
		"", 
		&support));


	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(
		&executor,
		&support.context, 
		2,
		&allocator));
    

	// ********************  create joint trajectory subscriber  *********************
	rcl_subscription_t joint_trajectory_sub;
	RCCHECK(rclc_subscription_init_default(
		&joint_trajectory_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(micro_custom_messages, msg, JointTrajectoryReducedArray),
		"joint_trajectory_reduced"));

	// allocate message memory
	micro_custom_messages__msg__JointTrajectoryReducedArray set_points_array_msg;	// allocate message memory
	set_points_array_msg.array_size = 400;	// max capacity
	set_points_array_msg.set_points.capacity = set_points_array_msg.array_size;
	set_points_array_msg.set_points.data = (micro_custom_messages__msg__JointTrajectoryReduced*) malloc(
		set_points_array_msg.set_points.capacity * sizeof(micro_custom_messages__msg__JointTrajectoryReduced));
	set_points_array_msg.set_points.size = 0;

	// add subscription to topic
	RCCHECK(rclc_executor_add_subscription(
		&executor, 
		&joint_trajectory_sub, 
		&set_points_array_msg,
		&trajectory_task_callback,
		ON_NEW_DATA));
	// **********************************************************************


	// ********************  create homing task subscriber  *********************
	rcl_subscription_t homing_sub;
	RCCHECK(rclc_subscription_init_default(
		&homing_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"task_homing"));

	// allocate message memory
	std_msgs__msg__Bool task_homing_msg;

	// add subscription to topic
	RCCHECK(rclc_executor_add_subscription(
		&executor, 
		&homing_sub, 
		&task_homing_msg,
		&homing_task_callback,
		ON_NEW_DATA));
	// **********************************************************************


	// ********************  create task ack publisher  *********************
	RCCHECK(rclc_publisher_init_default(
        &task_ack_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(micro_custom_messages, msg, TaskAck),
        "task_ack"));
	// **********************************************************************


	// spin the node
  	rclc_executor_spin(&executor);		// loop*

	// destroy nodes
	RCCHECK(rcl_subscription_fini(&joint_trajectory_sub, &node));
	RCCHECK(rcl_subscription_fini(&homing_sub, &node));
	RCCHECK(rcl_node_fini(&node));
	return;
}


void loop()
{
    // loop*
}