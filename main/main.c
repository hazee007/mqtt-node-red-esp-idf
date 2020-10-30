#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "connect.h"
#include "mqtt_client.h"
#include <dht.h>

#define TAG "MQTT"

xQueueHandle readingDHT;
TaskHandle_t taskHandle;

const uint32_t WIFI_CONNEECTED = BIT1;
const uint32_t MQTT_CONNECTED = BIT2;
const uint32_t MQTT_PUBLISHED = BIT3;

static const gpio_num_t dht_gpio = 18;
static const dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;

void mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  switch (event->event_id)
  {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    xTaskNotify(taskHandle, MQTT_CONNECTED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    xTaskNotify(taskHandle, MQTT_PUBLISHED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  mqtt_event_handler_cb(event_data);
}

void MQTTLogic(char *sensorReading)
{

  uint32_t command = 0;
  esp_mqtt_client_config_t mqttConfig = {
      // .uri = "mqtt://192.168.10.100:1883",
      .host = "192.168.10.100",
      .port = 1883};
  esp_mqtt_client_handle_t client = NULL;

  while (true)
  {
    xTaskNotifyWait(0, 0, &command, portMAX_DELAY);
    switch (command)
    {
    case WIFI_CONNEECTED:
      client = esp_mqtt_client_init(&mqttConfig);
      esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
      esp_mqtt_client_start(client);
      break;
    case MQTT_CONNECTED:
      esp_mqtt_client_subscribe(client, "/topic/my/subscription/1", 2);
      printf("sending data: %s \n", sensorReading);
      esp_mqtt_client_publish(client, "topic/my/publication/1", sensorReading, strlen(sensorReading), 2, false);
      break;
    case MQTT_PUBLISHED:
      esp_mqtt_client_stop(client);
      esp_mqtt_client_destroy(client);
      esp_wifi_stop();
      return;
    default:
      break;
    }
  }
}

void OnConnected(void *para)
{
  while (true)
  {
    char *sensorReading;
    if (xQueueReceive(readingDHT, &sensorReading, portMAX_DELAY))
    {
      ESP_ERROR_CHECK(esp_wifi_start());
      MQTTLogic(sensorReading);
    }
  }
}

void dht_test(void *pvParameters)
{
  char *json_str;
  int16_t temperature = 0;
  int16_t humidity = 0;

  cJSON *root;
  root = cJSON_CreateObject();
  while (1)
  {
    if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
    {

      cJSON_AddItemToObject(root, "temperature", cJSON_CreateNumber(temperature / 10));
      cJSON_AddItemToObject(root, "humidity", cJSON_CreateNumber(humidity / 10));
      json_str = cJSON_Print(root);
      printf("data: %s", json_str);
      xQueueSend(readingDHT, &json_str, 2000 / portTICK_PERIOD_MS);
      // printf("Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10);
      cJSON_Delete(root);
    }
    else
      printf("Could not read data from sensor\n");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void app_main()
{
  readingDHT = xQueueCreate(sizeof(int), 10);
  wifiInit();
  xTaskCreate(OnConnected, "handel comms", 1024 * 5, NULL, 5, &taskHandle);
  xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 10, NULL, 5, NULL);
}