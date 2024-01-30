#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "board_pinout.h"
#include "configuration.h"
#include "error_log.h"
#include "custom_messages.h"
#include "callback_functions.h"



void setup()
{
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(1000);

    // initialize shift register pinout
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_LATCH_PIN, OUTPUT);

	// micro-ROS setup
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;


	// allocate msg memory
	bool rc = micro_custom_messages__msg__SetPoint__Sequence__init(
    	set_point_sequence_msg,
    	set_point_sequence_size);

	if(rc == false){
		error_loop();
	}

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "motor_driver_node", "", &support));


	// create subscriber
    rcl_subscription_t subscriber;
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(micro_custom_messages, msg, SetPoint),
		"joint_position_micro"));


	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(
		&executor,
		&support.context, 
		1, 
		&allocator));
    
	RCCHECK(rclc_executor_add_subscription(
		&executor, 
		&subscriber, 
		&set_point_sequence_msg, 
		&subscription_callback,
		ON_NEW_DATA));

	// spin the node
  	rclc_executor_spin(&executor);

	// destroy nodes
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}


void loop()
{
    // loop
}