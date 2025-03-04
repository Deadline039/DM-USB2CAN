//
// Created by Deadline039 on 3/3/2025.
//

#ifndef DM_PROTOCOL_H
#define DM_PROTOCOL_H

#include "main.h"
extern uint32_t g_send_times;
extern uint32_t g_send_time_gap;

void DM_USB_Process(uint8_t *data, uint32_t len);
void DM_USB_SendCAN_Message(void);

#endif /* DM_PROTOCOL_H */