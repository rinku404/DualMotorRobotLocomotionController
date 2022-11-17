#include "espnow_manager.h"
#include <stdio.h>

//Sender    A8:03:2A:4D:14:64
//Receiver  78:21:84:81:5B:58

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x81, 0x5B, 0x58};

esp_now_peer_info_t peerInfo;
static char espnow_msg[40];

static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{

}

static void OnDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    //memcpy(&espnow_msg, incomingData, len);
    //printf("%s\n", espnow_msg.buff);

    for(int i = 0; i < len; i++)
    {
        printf("%c ", incomingData[i]);
    }
    printf("\n");
}

void ESPNowStartSender(void)
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        printf("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    //Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    //Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        printf("Failed to add peer");
        return;
    }
}

void ESPNowStartReceiver(void)
{
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        printf("Error initializing ESP-NOW");
        return;
    }

    //sprintf(espnow_msg.buff, "Hello!");

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataReceived);
}

void ESPNowSend(char* msg)
{
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msg, strlen(msg));
}