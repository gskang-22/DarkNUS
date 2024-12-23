/*
 * motor_control.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "motor_control.h"
#include "robot_config.h"

/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data
 * for target motor
 *
 */

void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data, uint8_t loopback) {

	double ang_diff = (setpoint - curr_pt);
	if (loopback){
		if (ang_diff > PI) {
			ang_diff -= 2 * PI;
		} else if (ang_diff < -PI) {
			ang_diff += 2 * PI;
		}
	}
//	float est_next_ang = ang_diff + (motor->raw_data.rpm * 2 * PI) * 0.01 / 60; //estimated angle in 10ms
//	if (est_next_ang > PI){
//		ang_diff -= 2 * PI;
//	} else if (est_next_ang < -PI){
//		ang_diff += 2 * PI;
//	}

	if (*prev_imu_data == imu_data) {
		return;}
	uint32_t time_mult;
	motor->angle_pid.last_time[1] = motor->angle_pid.last_time[0];
	motor->angle_pid.last_time[0] = get_microseconds();
	if (motor->angle_pid.last_time[0] <= motor->angle_pid.last_time[1]){
		time_mult = 1000 * 60 / GIMBAL_DELAY;

	} else {
		time_mult = TIMER_FREQ * 60 /
			(float) (motor->angle_pid.last_time[0] - motor->angle_pid.last_time[1]);
	}
	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = ang_diff;
	float rpm_pOut = motor->angle_pid.kp * ang_diff;
	float rpm_dOut = motor->angle_pid.kd * (motor->angle_pid.error[0] - motor->angle_pid.error[1]);

	float imu_ang_diff = imu_data - *prev_imu_data;
	imu_ang_diff = (imu_ang_diff > PI) ? imu_ang_diff - (2 * PI) :
			((imu_ang_diff < -PI) ? imu_ang_diff + (2*PI) : imu_ang_diff);
	float imu_rpm = (imu_ang_diff  * time_mult)/(2 * PI);
	*prev_imu_data = imu_data;
	motor->angle_pid.integral += motor->angle_pid.error[0]  * motor->angle_pid.ki;
	float_minmax(&motor->angle_pid.integral, motor->angle_pid.int_max, 0);
//	float rpm_iOut = motor->angle_pid.ki;

	float rpm_iOut = motor->angle_pid.integral;

	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_minmax(&motor->angle_pid.output, motor->angle_pid.max_out,0);

	if (fabs(motor->angle_pid.error[0]) < 0.06) {
	    motor->angle_pid.output = 0;
	}
	speed_pid(motor->angle_pid.output,imu_rpm, &motor->rpm_pid);

}


/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data
 * for target motor
 *
 */
void angle_pid(double setpoint, double curr_pt, motor_data_t *motor) {

	//update time
	uint32_t curr_time = get_microseconds();

	motor->angle_pid.last_time[1] = motor->angle_pid.last_time[0];
	motor->angle_pid.last_time[0] = curr_time;
	double delta_time = motor->angle_pid.last_time[0] - motor->angle_pid.last_time[1];


	if (delta_time == 0) {
		delta_time = 0.001;
	}

	//update error
	double angle_diff = setpoint - curr_pt;

	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = angle_diff;


	//calculate p,i,d for output
	double p = motor->angle_pid.kp * angle_diff;

	double derivative = (motor->angle_pid.error[0] - motor->angle_pid.error[1]) / delta_time;
	double d = derivative * motor->angle_pid.kd;

	motor->angle_pid.integral += motor->angle_pid.error[0] * motor->angle_pid.ki;
	float_minmax(&motor->angle_pid.integral, motor->angle_pid.int_max, 0);
//	double i = motor->angle_pid.integral * motor->angle_pid.ki;
	double i = motor->angle_pid.integral;

	double curr_output = p + i + d;
//	if (setpoint == 0) { // prevents robot from running forever
//		curr_output = 0;
//	}

	motor->angle_pid.output = curr_output;

	float_minmax(&motor->angle_pid.output, motor->angle_pid.max_out,0);


	speed_pid(motor->angle_pid.output, motor->raw_data.rpm, &motor->rpm_pid);
}


/*
 * Function for speed PID
 * For motors that might see constant torque, i.e. chassis motors
 * make sure an integral value is initialised (VERY SMALL, like 0.0001 or smaller)
 * as their systems usually have a steady state error
 *
 * @param setpoint target RPM
 * @param motor's current RPM
 * @param *pid pointer to the rpm_pid struct within the motor's data struct
 */
void speed_pid(double setpoint, double curr_pt, pid_data_t *pid) {

	//update time
    uint32_t curr_time = get_microseconds();

    pid->last_time[1] = pid->last_time[0];
    pid->last_time[0] = curr_time;
    double delta_time = pid->last_time[0] - pid->last_time[1];
//    double delta_time = (pid->last_time[0] - pid->last_time[1]) / 1000000.0;



    if (delta_time == 0) {
        delta_time = 0.001;
    }

    //update error
	double curr_error = setpoint - curr_pt;
	pid->error[1] = pid->error[0];
	pid->error[0] = curr_error;


	//calculate p,i,d for output
	double p = pid->kp * curr_error;

//	pid->integral += pid->ki * pid->error[0];
	pid->integral += curr_error * delta_time * pid->ki;

	if (fabs(curr_error) < 0.01) {  // Error threshold to consider "close enough"
	    pid->integral = 0;          // Reset integral term
	}

	float_minmax(&pid->integral, pid->int_max, 0);
//	double i = pid->ki;
	double i = pid->integral;


//	double derivative = (pid->error[0] - pid->error[1]);
    double derivative = (curr_error - pid->error[1]) / delta_time;
	double d = derivative * pid->kd;

	double curr_output = p + i + d;
	float_minmax(&pid->output, pid->max_out, 0);
	pid->output = curr_output;

//	if (setpoint == 0) { // prevents robot from running forever
//		pid->output = 0;
//	} else {
//		pid->output = curr_output;
//	}
}
// possible reasons for overcompensation: too high kp, too low kd, too high ki

/*
 * Function to send commands
 * To use, input the motor ids of the motors desired
 * it will then send the number in the rpm_pid.output of the motors
 * to the motors
 * Motors with the same CAN header will also be sent the last output value put in their
 * pid struct
 *
 * @param motor_all[] pointer to the array that contains the data for *all* the motors
 * @param id_one	id number of first motor to send can message to
 * @param id_two	id number of second motor to send can message to
 * @param id_three	id number of third motor to send can message to
 * @param id_four	id number of fourth motor to send can message to
 */
void motor_send_can(motor_data_t motor_all[],
		uint8_t id_one,
		uint8_t id_two,
		uint8_t id_three,
		uint8_t id_four) {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box;
	uint32_t temp_checker = 0;
	int16_t temp_converter;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
	if (id_one < 25 && id_one > 0) {
		temp_checker = temp_checker | 1 << (id_one - 1);
	}
	if (id_two < 25 && id_two > 0) {
		temp_checker = temp_checker | 1 << (id_two - 1);
	}
	if (id_three < 25 && id_three > 0) {
		temp_checker = temp_checker | 1 << (id_three - 1);
	}
	if (id_four < 25 && id_four > 0) {
		temp_checker = temp_checker | 1 << (id_four - 1);
	}

	if (temp_checker & 0x00000F) {
		CAN_tx_message.StdId = 0x200;
		temp_converter = motor_all[0x0].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x1].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x2].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x3].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x0000F0) {
		CAN_tx_message.StdId = 0x1FF;
		temp_converter = motor_all[0x4].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x5].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x6].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x7].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x000F00) {
		CAN_tx_message.StdId = 0x2FF;
		temp_converter = motor_all[0x8].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x9].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0xA].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0xB].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
#ifndef CHASSIS_MCU
	if (temp_checker & 0x00F000) {
		CAN_tx_message.StdId = 0x200;
		temp_converter = motor_all[0x0 + 12].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x1 + 12].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x2 + 12].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x3 + 12].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x0F0000) {
		CAN_tx_message.StdId = 0x1FF;
		temp_converter = motor_all[0x4 + 12].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x5 + 12].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x6 + 12].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x7 + 12].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0xF00000) {
		CAN_tx_message.StdId = 0x2FF;
		temp_converter = motor_all[0x8 + 12].output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x9 + 12].output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0xA + 12].output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0xB + 12].output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
#endif
}

void kill_can() {

}

/**
 * Limits the input float variable
 * @params motor_in: the pointer to the variable to be limited
 * @params motor_max: the positive maximum value for the variable
 */

void float_minmax(float *motor_in, float motor_max, float motor_min) {
	if (*motor_in > motor_max) {
		*motor_in = motor_max;
	} else if (*motor_in < -motor_max) {
		*motor_in = -motor_max;
	}

//	if (fabs(*motor_in) < motor_min) {
//		*motor_in = 0;
//	}

}

/**
 * Resets PID values for the motors using the motor_data_t struct
 * @params motor_data: pointer to the motor data struct
 */
void reset_pid(motor_data_t *motor_data) {

}



