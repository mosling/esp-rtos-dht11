#include "esp_common.h"

enum sensor_type{
	SENSOR_DHT11,SENSOR_DHT22
};

bool readDHT(enum sensor_type st, uint8 pin, int16_t *humidity, int16_t *temperature);
