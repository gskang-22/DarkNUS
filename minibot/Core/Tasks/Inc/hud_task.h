/*
 * hud_task.h
 *
 *  Created on: Jul 7, 2023
 *      Author: wx
 */

#ifndef TASKS_INC_HUD_TASK_H_
#define TASKS_INC_HUD_TASK_H_


void map_robot_id(uint16_t robot_id);
void hud_task(void *argument);
uint16_t clear_hud(uint8_t* tx_buffer);
void ref_send(uint8_t* tx_buffer, uint16_t tx_len);
uint8_t draw_char(uint16_t* buffer_size, uint8_t* tx_buffer, uint8_t* num_obj);
uint16_t draw_graphic_header(uint8_t* tx_buffer, uint16_t tx_len, uint8_t num_graphics);
uint16_t draw_crosshair(uint8_t modify,uint8_t* tx_buffer,uint8_t* num_graphics);
uint16_t draw_spinspin(uint8_t modify, uint8_t* tx_buffer);
uint16_t draw_gearing(uint8_t modify, uint8_t* tx_buffer);

#endif /* TASKS_INC_HUD_TASK_H_ */
