#ifndef ESPNOW_MANAGER_H
#define ESPNOW_MANAGER_H

#include "espnow_msg.h"

void ESPNowStart(void);
void ESPNowSend(espnow_msg_t* msg);

#endif