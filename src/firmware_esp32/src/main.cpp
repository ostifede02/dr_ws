#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int64.h>

#include "ShiftRegister74HC595.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif


// HARDWARE
#define PIN_STEP 1
#define PIN_DIR 2

// create a global shift register object
#define NUMBER_SHIFT_REG 1
#define I2S_DATA_PIN 21
#define I2S_CLOCK_PIN 16
#define I2S_LATCH_PIN 17

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);



// Function prototype:
void (* rclc_subscription_callback_t)(const void *);
void do_step(void);


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Subscriber object
rcl_subscription_t subscriber;



#define RCCHECK(fn)                 \
{                                   \
    rcl_ret_t temp_rc = fn;         \
    if ((temp_rc != RCL_RET_OK))    \
    {                               \
        error_loop();               \
    }                               \
}
#define RCSOFTCHECK(fn)             \
{                                   \
    rcl_ret_t temp_rc = fn;         \
    if ((temp_rc != RCL_RET_OK))    \
    {                               \
    }                               \
}

// Error handle loop
void error_loop()
{
    while (1)
    {
        delay(100);
    }
}

void setup()
{
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_LATCH_PIN, OUTPUT);
    

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

    // **** initialize reliable subscriber ****
    const char * topic_name = "joint_position_micro";
    // Get message type support
    const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64);
    // Initialize a reliable subscriber
    rcl_ret_t rc = rclc_subscription_init_default(
    &subscriber, &node,
    type_support, topic_name);

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));


    // Message object to receive publisher data
    std_msgs__msg__Int64 msg;
    // Add subscription to the executor
    rcl_ret_t rc = rclc_executor_add_subscription(
    &executor, &subscriber, &msg,
    &subscription_callback, ON_NEW_DATA);



    msg.data = 0;
}

void loop()
{
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}



// Implementation example:
void subscription_callback(const void * msgin)
{
    // Cast received message to used type
    const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;

    // Process message

}

void do_step(void)
{
    // digitalWrite(PIN_STEP, HIGH);
    sr.set(PIN_STEP, HIGH);
    delayMicroseconds(40);
    // digitalWrite(PIN_STEP, LOW);
    sr.set(PIN_STEP, LOW);
    delayMicroseconds(40);
    return;
}