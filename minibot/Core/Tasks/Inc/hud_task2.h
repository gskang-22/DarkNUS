/*
 * hud_task2.h
 *
 *  Created on: Aug 9, 2024
 *      Author: wx, cw
 */

#ifndef TASKS_INC_HUD_TASK2_H_
#define TASKS_INC_HUD_TASK2_H_

#include <cstdint>

class HUDTask {
public:
	void mapRobotId(uint16_t robot_id);
	uint16_t clearAllLayers(uint8_t* tx_buffer); // Clear all layers
	uint16_t clearOneLayer(uint8_t* tx_buffer, uint8_t layer); // Clear one layer

	void sendData(uint8_t* tx_buffer, uint16_t tx_len);
};

class HUDDraw : public HUDTask {
public:
	void drawInterface(uint8_t modify);
	void drawSpinSpin(uint8_t modify);
	void drawCrossHair(uint8_t modify);
};

void hudTask2();

#endif /* TASKS_INC_HUD_TASK2_H_ */
