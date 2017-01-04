#include "esp_common.h"
#include "dht.h"
#include "private_ssid_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "MQTTESP8266.h"
#include "MQTTClient.h"

xSemaphoreHandle wifi_alive;
xQueueHandle publish_queue;

#define PUB_MSG_LEN 16

typedef struct
{
    uint8 type;
    int16_t value;
} measurement;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map)
    {
    case FLASH_SIZE_4M_MAP_256_256:
        rf_cal_sec = 128 - 5;
        break;

    case FLASH_SIZE_8M_MAP_512_512:
        rf_cal_sec = 256 - 5;
        break;

    case FLASH_SIZE_16M_MAP_512_512:
    case FLASH_SIZE_16M_MAP_1024_1024:
        rf_cal_sec = 512 - 5;
        break;

    case FLASH_SIZE_32M_MAP_512_512:
    case FLASH_SIZE_32M_MAP_1024_1024:
        rf_cal_sec = 1024 - 5;
        break;

    default:
        rf_cal_sec = 0;
        break;
    }

    return rf_cal_sec;
}

const char *get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!wifi_get_macaddr(STATION_IF, my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9)
            x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9)
            x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

void wifi_task(void *pvParameters)
{
    uint8_t status;

    if (wifi_get_opmode() != STATION_MODE)
    {
        wifi_set_opmode(STATION_MODE);
        vTaskDelay(1000 / portTICK_RATE_MS);
        system_restart();
    }

    while (1)
    {
        os_printf("WiFi: Connecting to WiFi\n");
        wifi_station_connect();
        struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
        sprintf(config->ssid, WIFI_SSID);
        sprintf(config->password, WIFI_PASS);
        wifi_station_set_config(config);
        free(config);
        status = wifi_station_get_connect_status();
        int8_t retries = 30;
        while ((status != STATION_GOT_IP) && (retries > 0))
        {
            status = wifi_station_get_connect_status();
            if (status == STATION_WRONG_PASSWORD)
            {
                os_printf("WiFi: Wrong password\n");
                break;
            }
            else if (status == STATION_NO_AP_FOUND)
            {
                os_printf("WiFi: AP not found\n");
                break;
            }
            else if (status == STATION_CONNECT_FAIL)
            {
                os_printf("WiFi: Connection failed\n");
                break;
            }
            vTaskDelay(1000 / portTICK_RATE_MS);
            --retries;
        }

        if (status == STATION_GOT_IP)
        {
            os_printf("WiFi: Connected\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }

        while ((status = wifi_station_get_connect_status()) == STATION_GOT_IP)
        {
            xSemaphoreGive(wifi_alive);
            // os_printf("WiFi: Alive\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }

        os_printf("WiFi: Disconnected\n");
        wifi_station_disconnect();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void enqueueMessage(measurement *m)
{
    if (xQueueSend(publish_queue, m, 0) == pdFALSE)
    {
        os_printf("Publish queue overflow.\r\n");
    }
}

/* Demonstrating sending something to MQTT broker
   In this task we simply queue up messages in publish_queue. The MQTT task will dequeue the
   message and sent.
 */
void beat_task(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    measurement msg;
    int count = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_RATE_MS); // This is executed every 10000ms
        msg.type = 0;
        msg.value = count++;
        enqueueMessage(&msg);
    }
}

char *determinePublish(measurement *m)
{
    if (m->type == 2)
        return "temperature";

    if (m->type == 1)
        return "humidity";

    return "beat";
}


void mqtt_task(void *pvParameters)
{
    int ret;
    struct Network network;
    MQTTClient client = DefaultClient;
    char mqtt_client_id[20];
    unsigned char mqtt_buf[100];
    unsigned char mqtt_readbuf[100];
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

    NewNetwork(&network);
    while (1)
    {
        // Wait until wifi is up
        xSemaphoreTake(wifi_alive, portMAX_DELAY);

        // Unique client ID
        strcpy(mqtt_client_id, "ESP-");
        strcat(mqtt_client_id, get_my_id());

        os_printf("(Re)connecting to MQTT server %s ... ", MQTT_HOST);
        ret = ConnectNetwork(&network, MQTT_HOST, MQTT_PORT);
        if (!ret)
        {
            NewMQTTClient(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);
            data.willFlag = 0;
            data.MQTTVersion = 3;
            data.clientID.cstring = mqtt_client_id;
            data.username.cstring = MQTT_USER;
            data.password.cstring = MQTT_PASS;
            data.keepAliveInterval = 10;
            data.cleansession = 0;
            os_printf("Send MQTT connect ...");
            ret = MQTTConnect(&client, &data);
            if (!ret)
            {
                os_printf("ok.\r\n");
                // Subscriptions
                // MQTTSubscribe(&client, "/mytopic", QOS1, topic_received);
                // Empty the publish queue
                xQueueReset(publish_queue);
                while (1)
                {
                    // Publish all pending messages
                    char msg[PUB_MSG_LEN];
                    measurement m;
                    while (xQueueReceive(publish_queue, (void *)&m, 0) == pdTRUE)
                    {
                        MQTTMessage message;
                        snprintf(msg, PUB_MSG_LEN, "%d\0", m.value);
                        msg[PUB_MSG_LEN - 1] = '\0';
                        message.payload = msg;
                        message.payloadlen = strlen(msg);
                        message.dup = 0;
                        message.qos = QOS1;
                        message.retained = 0;
                        ret = MQTTPublish(&client, determinePublish(&m), &message);
                        if (ret != SUCCESS)
                            break;
                    }
                    // Receiving / Ping
                    ret = MQTTYield(&client, 1000);
                    if (ret == DISCONNECTED)
                    {
                        break;
                    }
                }
                os_printf("Connection broken, request restart\r\n");
            }
            else
            {
                os_printf("failed.\r\n");
            }
            DisconnectNetwork(&network);
        }
        else
        {
            os_printf("failed.\r\n");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    os_printf("MQTT task ended\r\n", ret);
    vTaskDelete(NULL);
}

void dht_task(void *pvParameters)
{
    // Set GPIO2 to output mode for DHT22
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
    //PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);

    int16_t h;
    int16_t t;
    measurement hmsg;
    measurement tmsg;
    bool r;

    hmsg.type = 1;
    tmsg.type = 2;
    while (1)
    {
        r = readDHT(SENSOR_DHT11, 2, &h, &t);
        if (true == r)
        {
            os_printf("Humidity   : %d Temperature: %d \n", h, t);
            hmsg.value = h;
            tmsg.value = t;
            enqueueMessage(&hmsg);
            enqueueMessage(&tmsg);
        }
        else
        {
            os_printf("error reading dht11 values ...\n");
        }

        // three seconds delay...
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    os_printf("SDK version     :%s\n", system_get_sdk_version());
    os_printf("ESP8266 chip ID :0x%x\n", system_get_chip_id());

    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(10, PUB_MSG_LEN);
    xSemaphoreTake(wifi_alive, 0); // take the default semaphore

    xTaskCreate(beat_task, "beat", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(mqtt_task, "mqtt", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(wifi_task, "wifi", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(dht_task, "dht11", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
}
