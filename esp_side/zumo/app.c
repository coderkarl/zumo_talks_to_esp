#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// See microros_ws/firmware/mcu_ws/install/share
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "driver/uart.h"
#include "driver/gpio.h"

#define BUF_SIZE 1024
#define ZUMO_UART_READ_WAIT_MS 50
#define ZUMO_UART_NUM UART_NUM_2

// Portability layer for ESP32
static uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

void zumo_init_serial(){
    ESP_ERROR_CHECK(uart_param_config(ZUMO_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ZUMO_UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // use default UART1 pins
    //ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(ZUMO_UART_NUM, BUF_SIZE, 0, 0, NULL, 0));
}

void zumo_write_bytes(uint8_t * buf, size_t len){
    uart_write_bytes(ZUMO_UART_NUM, (const char*)buf, len);
}

size_t zumo_read_bytes(uint8_t * buf, size_t len){
    size_t available_lenght;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(ZUMO_UART_NUM, (size_t*)&available_lenght));
    available_lenght = (available_lenght > len) ? len : available_lenght;
    return uart_read_bytes(ZUMO_UART_NUM, buf, available_lenght, 20 / portTICK_RATE_MS); // 20 / portTICK_RATE_MS or pdMS_TO_TICKS(ZUMO_UART_READ_WAIT_MS)
}

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t odom_data_publisher;
rcl_publisher_t prox_array_publisher;
rcl_publisher_t string_publisher;
rcl_subscription_t cmd_vel_subscriber;

std_msgs__msg__String output_str;
geometry_msgs__msg__Point32 odom_data;
geometry_msgs__msg__Twist cmd_vel;
std_msgs__msg__UInt8MultiArray prox_array;

#define WHEEL_BASE_METERS 0.1

float cumEncLeft = 0;
float cumEncRight = 0;
float cumYawDeg = 0.0;

float linear_vel_m = 0.0;
float angular_vel_deg = 0.0;

struct timespec last_cmd_ts;

uint64_t millis_since(struct timespec prev_ts)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    //sprintf(output_str.data.data, "dsec %ld, dmillis %ld", ts.tv_sec - prev_ts.tv_sec, (ts.tv_nsec - prev_ts.tv_nsec)/1000000);
    //output_str.data.size = strlen(output_str.data.data);
    //rcl_publish(&string_publisher, (const void*)&output_str, NULL);
    return (ts.tv_sec - prev_ts.tv_sec) * 1000 + (ts.tv_nsec - prev_ts.tv_nsec) / 1e6;
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {
        uint64_t cmd_per_msec = millis_since(last_cmd_ts);
        static int count = 0;
        count++;
        if(count % 10 == 0) {
          sprintf(output_str.data.data, "odom_timer_count_%d. millis_since_cmd %lld", count, cmd_per_msec);
		  output_str.data.size = strlen(output_str.data.data);
          rcl_publish(&string_publisher, (const void*)&output_str, NULL);
        }
        float dt = 0.05;
        
        if(cmd_per_msec > 1000)
        {
            linear_vel_m  = 0.0;
            angular_vel_deg = 0.0;
        }
        
        const char data_req_cmd[5] = {'A', '3', '/', '4', '/'};
        uart_write_bytes(ZUMO_UART_NUM, (const char *) data_req_cmd, 5);
        //uint8_t *zumo_data = (uint8_t *) malloc(BUF_SIZE); // Remember free(zumo_data); at end of function
        uint8_t zumo_data[BUF_SIZE];
        int len = 0;
        if(1)
        {
            len = zumo_read_bytes(zumo_data, BUF_SIZE);
            uart_flush(ZUMO_UART_NUM);
        }
        
        if(1)
        {
          sprintf(output_str.data.data, "len %d, zumo data %d %d %d %d", len, zumo_data[0], zumo_data[1], zumo_data[2], zumo_data[3]);
          output_str.data.size = strlen(output_str.data.data);
          rcl_publish(&string_publisher, (const void*)&output_str, NULL);
        }
        
        int8_t delta_enc_left = 0;
        int8_t delta_enc_right = 0;
        int16_t yaw_rate_raw = 0;
        uint8_t prox_left_counts = 0;
        uint8_t prox_front_left_counts = 0;
        uint8_t prox_front_right_counts = 0;
        uint8_t prox_right_counts = 0;
        if(len >= 8)
        {
            delta_enc_left = zumo_data[0];
            delta_enc_right = zumo_data[1];
            yaw_rate_raw = (zumo_data[2] << 8 )+ zumo_data[3];
            prox_left_counts = zumo_data[4];
            prox_front_left_counts = zumo_data[5];
            prox_front_right_counts = zumo_data[6];
            prox_right_counts = zumo_data[7];
        }
        
        odom_data.x = delta_enc_left;
        odom_data.y = delta_enc_right;
        odom_data.z = yaw_rate_raw * 0.0091;
        RCSOFTCHECK(rcl_publish( &odom_data_publisher, (const void *) &odom_data, NULL));
        
        prox_array.data.data[0] = prox_left_counts;
        prox_array.data.data[1] = prox_front_left_counts;
        prox_array.data.data[2] = prox_front_right_counts;
        prox_array.data.data[3] = prox_right_counts;
        RCSOFTCHECK(rcl_publish( &prox_array_publisher, (const void *) &prox_array, NULL));
        
        //free(zumo_data);

		// Fill the message timestamp
		//struct timespec ts;
		//clock_gettime(CLOCK_REALTIME, &ts);
		//outcoming_ping.stamp.sec = ts.tv_sec;
		//outcoming_ping.stamp.nanosec = ts.tv_nsec;
	}
}

void cmd_vel_subscription_callback(const void * msgin)
{
    if(millis_since(last_cmd_ts) < 100)
    {
        return;
    }
	const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;

	linear_vel_m = msg->linear.x;
    angular_vel_deg = msg->angular.z*180.0/3.14159;

    if(linear_vel_m > 1.2)
    {
        linear_vel_m = 1.2;
    }
    else if(linear_vel_m < -1.2)
    {
        linear_vel_m = -1.2;
    }
    if(angular_vel_deg > 120)
    {
        angular_vel_deg = 120;
    }
    else if(angular_vel_deg < -120)
    {
        angular_vel_deg = -120;
    }
    
    uint8_t cmd_bytes[7] = {'A', '1', '/', '1', '/', 0, 0};
    int8_t linear_byte = (int8_t)(linear_vel_m*100);
    int8_t angular_byte = (int8_t)(angular_vel_deg);
    cmd_bytes[5] = linear_byte;
    cmd_bytes[6] = angular_byte;
    if(1)
    {
       sprintf(output_str.data.data, "linear_byte %d, cmd_bytes[5] %d, angular_byte %d, cmd_bytes[6] %d", linear_byte, cmd_bytes[5], angular_byte, cmd_bytes[6]);
       output_str.data.size = strlen(output_str.data.data);
       rcl_publish(&string_publisher, (const void*)&output_str, NULL);
     }
    uart_write_bytes(ZUMO_UART_NUM, (const char *) cmd_bytes, 7);
    
    clock_gettime(CLOCK_REALTIME, &last_cmd_ts);
}

void appMain(void *argument)
{
    printf("HELLO****************\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "uros_zumo_node", "", &support));

	// Create a reliable ping publisher
	RCCHECK(rclc_publisher_init_default(&string_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/zumo_output"));

	// Create a best effort pong publisher
	RCCHECK(rclc_publisher_init_default(&odom_data_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/odom_data"));
     
     // Create a zumo_prox publisher
	RCCHECK(rclc_publisher_init_default(&prox_array_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/zumo_prox"));   

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

	// Create a 50 msec odom timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));


	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer)); // TEMP DISABLE TIMER ODOM CALLBACK TO DEBUG CMD VEL
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel,
		&cmd_vel_subscription_callback, ON_NEW_DATA));

	// Fill the array with a known sequence
	output_str.data.data = (char * ) malloc(STRING_BUFFER_LEN * sizeof(char));
	output_str.data.size = 0;
	output_str.data.capacity = STRING_BUFFER_LEN;
    
    prox_array.data.data = (uint8_t*) malloc(4*sizeof(uint8_t));
    prox_array.data.size = 4;
    prox_array.data.capacity = 4;
    
    zumo_init_serial();

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&odom_data_publisher, &node));
    RCCHECK(rcl_publisher_fini(&prox_array_publisher, &node));
	RCCHECK(rcl_publisher_fini(&string_publisher, &node));
	RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
    free(output_str.data.data);
    free(prox_array.data.data);
}
