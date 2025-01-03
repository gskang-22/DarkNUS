/*
 * hud_task.c
 *
 *  Created on: Jul 7, 2023
 *      Author: wx
 */

// todo: Implement crosshair drawing (if you want)

#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "hud_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"
#include "hud_constants.h"

static uint16_t g_client_id = 0;
extern int g_spinspin_mode;
extern uint8_t remote_raw_data[18];
extern TaskHandle_t referee_processing_task_handle;
extern uint8_t g_ref_tx_seq;
extern uint8_t control_mode;
extern uint8_t g_safety_toggle;
extern uint8_t launcher_safety_toggle;
extern speed_shift_t gear_speed;
#define HUD_BUFFER_SIZE

extern uint32_t reset_debounce_time;
extern uint32_t reset_start_time;


extern ref_game_robot_data_t ref_robot_data;
extern enum feeder_state_e feeder_state;
uint16_t draw_feeder_state(uint8_t modify, uint8_t* tx_buffer);

void map_robot_id(uint16_t robot_id){
	switch (robot_id) {
		case 1:
			g_client_id = 0x101;
			break;
		case 2:
			g_client_id = 0x102;
			break;
		case 3:
			g_client_id = 0x103;
			break;
		case 4:
			g_client_id = 0x104;
			break;
		case 5:
			g_client_id = 0x105;
			break;
		case 6:
			g_client_id = 0x106;
			break;

		case 101:
			g_client_id = 0x165;
			break;
		case 102:
			g_client_id = 0x166;
			break;
		case 103:
			g_client_id = 0x167;
			break;
		case 104:
			g_client_id = 0x168;
			break;
		case 105:
			g_client_id = 0x169;
			break;
		case 106:
			g_client_id = 0x16A;
			break;
		default:
			g_client_id = 0;
			break;

		}
}

uint16_t draw_reset(uint8_t modify, uint8_t* tx_buffer);

void hud_task(void *argument) {
	uint8_t tx_buffer[256];
	uint8_t num_graphics;
	uint16_t tx_len = 0;

	graphic_data_struct_t graphic_data;
	ref_frame_header_t send_header;
	uint8_t char_buffer[30];

	enum drawings {
		spinspin, gearing, robot_state, motor_fault, refresh, crosshair

	};
	while (ref_robot_data.robot_id == 0) {
		vTaskDelay(10);
	}
	map_robot_id(ref_robot_data.robot_id);
	uint32_t refresh_timer = HAL_GetTick();

	uint8_t draw_state = spinspin;
	//draw all images
//	vTaskDelay(1000);
	tx_len = clear_hud(tx_buffer);
	ref_send(tx_buffer, tx_len);
	vTaskDelay(150);
	draw_spinspin(0,tx_buffer);
	vTaskDelay(150);
	draw_feeder_state(0, tx_buffer);
	vTaskDelay(150);
	draw_gearing(0, tx_buffer);
	vTaskDelay(150);

	while (1) {
		if (HAL_GetTick() - refresh_timer > 120000){
			draw_state = refresh;
			refresh_timer = HAL_GetTick();
		}

		map_robot_id(ref_robot_data.robot_id);
		tx_len = 0;
		static uint8_t reset = 0;
		if (HAL_GetTick() - reset_start_time < 5000){
			draw_reset(0, tx_buffer);
			reset = 1;
			vTaskDelay(150);
		} else if (reset == 1) {
			reset = 0;
			draw_state = refresh;
		}


		switch (draw_state) {
		case spinspin:
			draw_spinspin(1,tx_buffer);
			draw_state = gearing;
			break;
		case gearing:
			draw_gearing(1,tx_buffer);
			draw_state = robot_state;
			break;
		case robot_state:
			draw_feeder_state(1, tx_buffer);
			draw_state = spinspin;
			break;
		case motor_fault:

			draw_state = spinspin;
			break;
		case refresh:
			tx_len = clear_hud(tx_buffer);
			ref_send(tx_buffer, tx_len);
			vTaskDelay(150);
			draw_spinspin(0,tx_buffer);
			vTaskDelay(150);
			draw_feeder_state(0, tx_buffer);
			vTaskDelay(150);
			draw_gearing(0,tx_buffer);
			draw_state = spinspin;
			break;
		default:
			break;
		}
		vTaskDelay(150);
	}
}
uint16_t clear_hud(uint8_t* tx_buffer){

	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x100;
	ref_delete->graphic_layer = 9;
	ref_delete->graphic_operation = 2;
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = ref_robot_data.robot_id;
	return (7+sizeof(ref_delete_graphic_t));
}


void ref_send(uint8_t* tx_buffer, uint16_t tx_len){
	append_CRC16_check_sum(tx_buffer, tx_len + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	static uint8_t tx_dma_buffer[256];
	//separate buffer to prevent touching dma buffer while tx is ongoing
	memcpy(tx_dma_buffer, tx_buffer, tx_len+2);
	HAL_UART_Transmit_DMA(&huart6, tx_dma_buffer, tx_len + 2);
}
uint8_t draw_char(uint16_t* buffer_size, uint8_t* tx_buffer, uint8_t* num_obj){
//	if (num_obj >= 7){
//		return num_obj;
//	}
//	else if (&buffer_size > HUD_BUFFER_SIZE){
//		return num_obj;
//	}
}

uint16_t draw_graphic_header(uint8_t* tx_buffer, uint16_t tx_len, uint8_t num_graphics){
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ tx_len;
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + 7);
	if (num_graphics == 1){
		graphic_header->cmd_ID = 0x101;
	}
	if (num_graphics == 2){
		graphic_header->cmd_ID = 0x102;
	}
	if (num_graphics == 5){
		graphic_header->cmd_ID = 0x103;
	}
	if (num_graphics == 7){
		graphic_header->cmd_ID = 0x104;
	}
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;
	return tx_len+sizeof(ref_frame_header_t)+sizeof(ref_inter_robot_data_t);
}

//TODO: improve or correct Crosshair Drawing code
//uint16_t draw_crosshair(uint8_t modify, uint8_t* tx_buffer) {
//    uint32_t curr_pos = 0;
//    ref_frame_header_t* send_header = (ref_frame_header_t*) tx_buffer;
//    ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));
//    graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t));
//
//    // Prepare the header
//    send_header->start_frame = 0xA5;
//    send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
//    send_header->seq = g_ref_tx_seq++;
//    send_header->data_length = sizeof(ref_inter_robot_data_t) + (2 * sizeof(graphic_data_struct_t)); // 2 lines for the crosshair
//    append_CRC8_check_sum((uint8_t*)send_header, 5);
//
//    // Set up the graphic header
//    graphic_header->cmd_ID = 0x120; // Arbitrary ID for crosshair
//    graphic_header->send_ID = ref_robot_data.robot_id;
//    graphic_header->receiver_ID = g_client_id;
//
//    curr_pos += sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t);
//
//    // Define horizontal line
//    graphic_data->graphic_name[0] = 'C';
//    graphic_data->graphic_name[1] = 'H';
//    graphic_data->graphic_name[2] = '1';
//    graphic_data->layer = 2;
//    graphic_data->color = 5; // Cyan
//    graphic_data->graphic_operation = modify ? 1 : 0; // Modify or add
//    graphic_data->width = 2;
//    graphic_data->start_x = 500; // Example coordinates
//    graphic_data->start_y = 400;
//    graphic_data->end_x = 700;
//    graphic_data->end_y = 400;
//    memcpy(tx_buffer + curr_pos, graphic_data, sizeof(graphic_data_struct_t));
//    curr_pos += sizeof(graphic_data_struct_t);
//
//    // Define vertical line
//    graphic_data->graphic_name[0] = 'C';
//    graphic_data->graphic_name[1] = 'H';
//    graphic_data->graphic_name[2] = '2';
//    graphic_data->start_x = 600; // Centered x-coordinate
//    graphic_data->start_y = 300;
//    graphic_data->end_x = 600;
//    graphic_data->end_y = 500;
//    memcpy(tx_buffer + curr_pos, graphic_data, sizeof(graphic_data_struct_t));
//    curr_pos += sizeof(graphic_data_struct_t);
//
//    return curr_pos;
//}


uint16_t draw_spinspin(uint8_t modify, uint8_t* tx_buffer) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	ref_frame_header_t* send_header = (ref_frame_header_t*) tx_buffer;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t));
	curr_pos = 0;
	char char_buffer[30];
	if (g_spinspin_mode == 0) {
		graphic_data->color = 4; //orange
		char_len = snprintf((char*) char_buffer, 30, "SPIN OFF");
	} else {
		graphic_data->color = 3; //orange
		char_len = snprintf((char*) char_buffer, 30, "SPIN ON");
	}
	send_header->start_frame = 0xA5;
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header->seq = g_ref_tx_seq++;
//	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 graphic
	graphic_header->cmd_ID = 0x110;
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 0;
	graphic_data->graphic_name[1] = 0;
	graphic_data->graphic_name[2] = 1;
	graphic_data->layer = 3;
	//draw number
	if (modify == 1) {
		graphic_data->operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data->operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data->graphic_type = 7; // char
	graphic_data->details_a = 30; // font size
	graphic_data->details_b = char_len; //number of decimal places
	graphic_data->width = 7; //line width
	graphic_data->layer = 0;
	//assuming 1920x1080? need check
	graphic_data->start_x = 50;
	graphic_data->start_y = 600;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
	return curr_pos+2;
}



uint16_t draw_feeder_state(uint8_t modify, uint8_t* tx_buffer) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	ref_frame_header_t* send_header = (ref_frame_header_t*) tx_buffer;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t));
	curr_pos = 0;
	char char_buffer[30];
	switch(feeder_state){

	case FEEDER_JAM:
		graphic_data->color = GRAPHIC_COLOUR_YELLOW; //orange
		char_len = snprintf((char*) char_buffer, 30, "JAMMED");
		break;

	case FEEDER_OVERHEAT:
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED; //orange
		char_len = snprintf((char*) char_buffer, 30, "OVERHEAT");
		break;

	case FEEDER_STANDBY:
		graphic_data->color = GRAPHIC_COLOUR_GREEN; //orange
		char_len = snprintf((char*) char_buffer, 30, "STANDBY");
		break;

	case FEEDER_SPINUP:
		graphic_data->color = GRAPHIC_COLOUR_GREEN; //orange
		char_len = snprintf((char*) char_buffer, 30, "FIRING");
		break;

	case FEEDER_FIRING:
	case FEEDER_STEP:
	case FEEDER_FREE:
	default:
		graphic_data->color = GRAPHIC_COLOUR_GREEN; //orange
		char_len = snprintf((char*) char_buffer, 30, "FIRING");
		break;

	}
	send_header->start_frame = 0xA5;
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header->seq = g_ref_tx_seq++;
//	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 graphic
	graphic_header->cmd_ID = 0x110;
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 0;
	graphic_data->graphic_name[1] = 'F';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 3;
	//draw number
	if (modify == 1) {
		graphic_data->operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data->operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data->graphic_type = 7; // char
	graphic_data->details_a = 30; // font size
	graphic_data->details_b = char_len; //number of decimal places
	graphic_data->width = 7; //line width
	graphic_data->layer = 0;
	//assuming 1920x1080? need check
	graphic_data->start_x = 50;
	graphic_data->start_y = 700;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
	return curr_pos+2;
}
//
//void draw_gearing(uint8_t modify) {
//
//	uint32_t curr_pos = 0;
//	uint8_t char_len = 0;
//	curr_pos = 0;
//	graphic_data.color = 6; //CYAN
//	char_len = snprintf((char*) char_buffer, 30, "GEAR %d", gear_speed.curr_gear);
//	send_header.start_frame = 0xA5;
//	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
//	send_header.seq = g_ref_tx_seq++;
//	send_header.data_length = sizeof(ref_inter_robot_data_t)
//			+ sizeof(graphic_data_struct_t) + char_len;
//	send_header.seq = g_ref_tx_seq++;
//	memcpy(tx_buffer + curr_pos, &send_header, 7);
//	curr_pos += sizeof(ref_frame_header_t);
//	append_CRC8_check_sum(tx_buffer, 5);
//
//	//for drawing 1 graphic
//	graphic_header.cmd_ID = 0x110;
//	//send to self
//	graphic_header.send_ID = ref_robot_data.robot_id;
//	graphic_header.receiver_ID = g_client_id;
//	memcpy(tx_buffer + curr_pos, &graphic_header,
//			sizeof(ref_inter_robot_data_t));
//	curr_pos += sizeof(ref_inter_robot_data_t);
//	//self set number for identification purposes only
//	graphic_data.graphic_name[0] = 0;
//	graphic_data.graphic_name[1] = 0;
//	graphic_data.graphic_name[2] = 2;
//	graphic_data.layer = 0;
//	//draw number
//	if (modify == 1) {
//		graphic_data.operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
//	} else {
//		graphic_data.operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
//	}
//	graphic_data.graphic_type = 7; // char
//	graphic_data.start_angle = 30; // font size
//	graphic_data.end_angle = char_len; //number of decimal places
//	graphic_data.width = 7; //line width
//	graphic_data.layer = 0;
//	//assuming 1920x1080? need check
//	graphic_data.start_x = 50;
//	graphic_data.start_y = 650;
//	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
//	curr_pos += sizeof(graphic_data_struct_t);
//	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
//	curr_pos += char_len;
//
//	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
//	while (huart6.gState != HAL_UART_STATE_READY) {
//		vTaskDelay(1);
//	}
//	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
//}
//

uint16_t draw_gearing(uint8_t modify, uint8_t* tx_buffer) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	ref_frame_header_t* send_header = (ref_frame_header_t*) tx_buffer;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t));
	curr_pos = 0;
	char char_buffer[30];
	graphic_data->color = GRAPHIC_COLOUR_CYAN; //CYAN
	char_len = snprintf((char*) char_buffer, 30, "GEAR %d", gear_speed.curr_gear);
	send_header->start_frame = 0xA5;
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header->seq = g_ref_tx_seq++;
//	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 graphic
	graphic_header->cmd_ID = 0x110;
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 0;
	graphic_data->graphic_name[1] = 'G';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 4;
	//draw number
	if (modify == 1) {
		graphic_data->operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data->operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data->graphic_type = 7; // char
	graphic_data->details_a = 30; // font size
	graphic_data->details_b = char_len; //number of decimal places
	graphic_data->width = 7; //line width
	graphic_data->layer = 0;
	//assuming 1920x1080? need check
	graphic_data->start_x = 50;
	graphic_data->start_y = 650;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
	return curr_pos+2;
}


uint16_t draw_reset(uint8_t modify, uint8_t* tx_buffer) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	ref_frame_header_t* send_header = (ref_frame_header_t*) tx_buffer;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t));
	curr_pos = 0;
	char char_buffer[30];
	graphic_data->color = 4; //orange
	char_len = snprintf((char*) char_buffer, 30, "RESET");
	send_header->start_frame = 0xA5;
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header->seq = g_ref_tx_seq++;
//	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 graphic
	graphic_header->cmd_ID = 0x110;
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 0;
	graphic_data->graphic_name[1] = 0;
	graphic_data->graphic_name[2] = 1;
	graphic_data->layer = 3;
	//draw number
	if (modify == 1) {
		graphic_data->operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data->operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data->graphic_type = 7; // char
	graphic_data->details_a = 100; // font size
	graphic_data->details_b = char_len; //number of decimal places
	graphic_data->width = 7; //line width
	graphic_data->layer = 0;
	//assuming 1920x1080? need check
	graphic_data->start_x = 250;
	graphic_data->start_y = 600;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
	return curr_pos+2;
}

