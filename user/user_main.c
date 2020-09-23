/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "math.h"
#include "wifi.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "driver/i2c_master.h"
#include "driver/I2C.h"
#include "driver/BME280.h"

#define GAddr 0x4A

void ICACHE_FLASH_ATTR init_gy49() {
  INFO("i2c_gpio_init\n");
  i2c_master_gpio_init();
  INFO("i2c_init\n");
  i2c_master_init();

  /*
	// Start I2C Transmission
	uint8_t i2c_addr_write = (GAddr << 1);
	i2c_master_start();
	i2c_master_writeByte(i2c_addr_write);
	bool r = i2c_master_checkAck();
        INFO("1:i2c_master_checkAck %d\n", r);

        // Select configuration register
	i2c_master_writeByte(0x02);
        r = i2c_master_checkAck();
        INFO("2:i2c_master_checkAck %d\n", r);
        // Continuous mode, Integration time = 800 ms
	i2c_master_writeByte(0x40);
        r = i2c_master_checkAck();
        INFO("3:i2c_master_checkAck %d\n", r);
	// Stop I2C transmission
	i2c_master_stop();
	*/
}

float ICACHE_FLASH_ATTR read_gy49() {
  unsigned int data[2];
  uint8_t i2c_addr_write = (GAddr << 1);
  uint8_t i2c_addr_read = (GAddr << 1 | 1);

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_write);
  uint8_t r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_writeByte(0x03);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_read);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d ir\n");

  data[0] = i2c_master_readByte();
  i2c_master_send_nack();

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_write);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_writeByte(0x04);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_read);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  data[1] = i2c_master_readByte();
  i2c_master_send_nack();


  i2c_master_stop();

  INFO("READ lumi byte 1 %d\n", data[0]);
  INFO("READ lumi byte 2 %d\n", data[1]);

  // Convert the data to lux
  int exponent = (data[0] & 0xF0) >> 4;
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);

  INFO("exponent %d\n", exponent);
  INFO("mantissa %d\n", mantissa);
  float luminance = pow(2, exponent) * mantissa * 0.045;
  return luminance;
}

uint8 read_ChipId() {
  uint8_t i2c_addr_write = (0x76 << 1);
  uint8_t i2c_addr_read = (0x76 << 1 | 1);
  uint8 data;

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_write);
  uint8_t r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_writeByte(0xD0);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d \n");

  i2c_master_start();

  i2c_master_writeByte(i2c_addr_read);
  r = i2c_master_checkAck();
  INFO("Received Ack: %d ir\n");

  data = i2c_master_readByte();
  i2c_master_send_nack();

  i2c_master_stop();

  return data;
}


MQTT_Client mqttClient;
static void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status) {
  if (status == STATION_GOT_IP) {
    MQTT_Connect(&mqttClient);
  } else {
    MQTT_Disconnect(&mqttClient);
  }
}
static void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args) {
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Connected\r\n");
  //MQTT_Subscribe(client, "/mqtt/topic/0", 0);
  //MQTT_Subscribe(client, "/mqtt/topic/1", 1);
  //MQTT_Subscribe(client, "/mqtt/topic/2", 2);

  //MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
  //MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
  //MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);
  //

  int voltage = system_adc_read();
  char str[5];
  os_sprintf(str, "%d", voltage);
  INFO("Voltage: %s\n", str);

  MQTT_Publish(client, "/mqtt/topic/0", str, 6, 0, 0);


  // read luminance
  float l = read_gy49();
  int l1 = l;
  int l2 = (l - l1) * 1000;
  INFO("Lux: %d.%d", l1, l2);
  char lstr[20];
  os_sprintf(lstr, "%d.%d", l1, l2);
  MQTT_Publish(client, "/mqtt/topic/1", lstr, 6, 0, 0);


  // bme280 stuff
  //
  sint32 t;
  sint32 h;
  sint32 p;
  BME280_ReadAll(&t, &p, &h);
  INFO("Temperature: %d.%d\n", t / 100, (t % 100) * 100);


}

static void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args) {
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Disconnected\r\n");
}

static void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args) {
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Published\r\n");
}

static void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len) {
  char *topicBuf = (char*)os_zalloc(topic_len + 1),
  *dataBuf = (char*)os_zalloc(data_len + 1);

  MQTT_Client* client = (MQTT_Client*)args;
  os_memcpy(topicBuf, topic, topic_len);
  topicBuf[topic_len] = 0;
  os_memcpy(dataBuf, data, data_len);
  dataBuf[data_len] = 0;
  INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
  os_free(topicBuf);
  os_free(dataBuf);
}

void ICACHE_FLASH_ATTR print_info() {
  INFO("\r\n\r\n[INFO] BOOTUP...\r\n");
  INFO("[INFO] SDK: %s\r\n", system_get_sdk_version());
  INFO("[INFO] Chip ID: %08X\r\n", system_get_chip_id());
  INFO("[INFO] Memory info:\r\n");
  system_print_meminfo();

  INFO("[INFO] -------------------------------------------\n");
  INFO("[INFO] Build time: %s\n", BUID_TIME);
  INFO("[INFO] -------------------------------------------\n");

}


static void ICACHE_FLASH_ATTR app_init(void) {
  uart_init(BIT_RATE_115200, BIT_RATE_115200);
  print_info();
  MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
  //MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

  if (!MQTT_InitClient(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE, MQTT_CLEAN_SESSION)) {
    INFO("Failed to initialize properly. Check MQTT version.\r\n");
    return;
  }
  //MQTT_InitClient(&mqttClient, "client_id", "user", "pass", 120, 1);
//  MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
//  MQTT_OnConnected(&mqttClient, mqttConnectedCb);
//  MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
//  MQTT_OnPublished(&mqttClient, mqttPublishedCb);
//  MQTT_OnData(&mqttClient, mqttDataCb);

//  WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);

  // init gy49
  init_gy49();

  float l = read_gy49();
  int l1 = l;
  int l2 = (l - l1) * 1000;
  INFO("Lux: %d.%d", l1, l2);

  int r = BME280_Init(BME280_OS_T_16, BME280_OS_P_16, BME280_OS_H_16, BME280_FILTER_16, BME280_MODE_NORMAL, BME280_TSB_05);
  
  uint8 data = read_ChipId();
//  I2C_ReadData(0x76 << 1, 0xD0, &data, 1);
  INFO("READ Chip ID: %d", data);
//  	
// INFO("init BME: %d\n", r);

  // bme280 stuff
  //
  sint32 t;
  sint32 h;
  sint32 p;
  BME280_ReadAll(&t, &p, &h);
  INFO("Temperature: %d.%d\n", t / 100, (t % 100) * 100);
}

void user_init(void) {
  system_init_done_cb(app_init);
}
