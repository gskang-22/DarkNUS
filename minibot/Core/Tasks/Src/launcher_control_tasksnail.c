/*
 * launcher_control_task.c
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "launcher_control_task.h"

#define SNAIL_PWM 1500  //500-1500
#define SNAIL_WINDDOWN			0
#define SNAIL_WINDUP			0

extern EventGroupHandle_t launcher_event_group;

extern motor_data_t can_motors[24];
extern gun_control_t launcher_ctrl_data;

extern referee_limit_t referee_limiters;

static uint32_t start_time = 0;
static uint32_t clear_time = 0;
static uint8_t unjamming = 0;
uint8_t firing = 0;
uint32_t stop_snail, start_snail;

uint16_t pwm_val = SNAIL_PWM;

extern QueueHandle_t telem_motor_queue;

void launcher_control_task(void *argument) {
	TickType_t start_time;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //start PWM (added)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //start PWM (added)
	htim1.Instance->CCR1 = 999; //don't go over 1.5ms
	htim1.Instance->CCR2 = 999;
	vTaskDelay(2);
	htim1.Instance->CCR1 = 1999; //don't go over 1.5ms
	htim1.Instance->CCR2 = 1999;
	vTaskDelay(2);
	htim1.Instance->CCR1 = 999; //don't go over 1.5ms
	htim1.Instance->CCR2 = 999;

	while (1) {
		//event flags!
//		xEventGroupWaitBits(launcher_event_group, 0b111, pdTRUE, pdFALSE,portMAX_DELAY);
		status_led(4, on_led);
		start_time = xTaskGetTickCount();
		if (launcher_ctrl_data.enabled) {
			launcher_control(can_motors + LFRICTION_MOTOR_ID - 1,
					can_motors + RFRICTION_MOTOR_ID - 1,
					can_motors + FEEDER_MOTOR_ID - 1);

		} else {
			htim1.Instance->CCR1 = 999;
			htim1.Instance->CCR2 = 999;
			can_motors[FEEDER_MOTOR_ID - 1].rpm_pid.output = 0;
			motor_send_can(can_motors, FEEDER_MOTOR_ID,0,0, 0);
		}
		status_led(4, off_led);
		//vTaskDelay(CHASSIS_DELAY);
//		xEventGroupClearBits(launcher_event_group, 0b111);
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}

}

void launcher_control(motor_data_t *left_friction_motor,
		motor_data_t *right_friction_motor, motor_data_t *feeder) {

	int16_t feeder_output = 0;
//	speed_pid(launcher_ctrl_data.gun_feeding_speed, feeder->raw_data.rpm, &feeder->rpm_pid);
//	motor_send_can(can_motors, FEEDER_MOTOR_ID, 0,
//			0, 0);

	if (launcher_ctrl_data.projectile_speed != 0) {
		//start snail
		htim1.Instance->CCR1 = pwm_val; //don't go over 1.5ms
		htim1.Instance->CCR2 = pwm_val;
		if (firing == 0){
			start_snail = HAL_GetTick();
			firing = 1;
		}

		if ((HAL_GetTick() - start_snail) > SNAIL_WINDUP) {
			//start feeder
			if (fabs(feeder->raw_data.torque) > FEEDER_JAM_TORQUE) {
				unjamming = 1;
				start_time = HAL_GetTick();
			}
			if (unjamming == 1) {
				if (start_time + FEEDER_UNJAM_TIME < HAL_GetTick()) {
					unjamming = 0;
					feeder_output = launcher_ctrl_data.gun_feeding_speed;
				} else {
					feeder_output = FEEDER_UNJAM_SPD;
				}
			} else {
				feeder_output = launcher_ctrl_data.gun_feeding_speed;
			}
		}
		stop_snail = HAL_GetTick();
	}

	else {
		//turn off feeder
		feeder_output = 0;
		if ((HAL_GetTick() - stop_snail) > SNAIL_WINDDOWN) {
			//turn off snail
			htim1.Instance->CCR1 = 999;
			htim1.Instance->CCR2 = 999;
		}
		start_snail = HAL_GetTick();
		firing = 0;
	}

	if (feeder_output == 0) {
		feeder->rpm_pid.output = 0;
//		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
	} else {
//		speed_pid(feeder_output * 36, feeder->raw_data.rpm, &feeder->rpm_pid);
		speed_pid(feeder_output * 36,feeder->raw_data.rpm, &feeder->rpm_pid);
	}

	motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
			RFRICTION_MOTOR_ID, 0);

}

