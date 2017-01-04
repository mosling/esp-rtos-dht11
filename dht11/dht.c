#include "dht.h"
#include "gpio.h"

#define DHT_TIMER_INTERVAL 2
#define DHT_DATA_BITS 40

// #define DEBUG_DHT
#ifdef DEBUG_DHT
#define debug(fmt, ...) printf("%s" fmt "\n", "dht: ", ##__VA_ARGS__);
#else
#define debug(fmt, ...) /* (do nothing) */
#endif

static bool dht_await_pin_state(uint8_t pin, uint32_t timeout,
                                bool expected_pin_state, uint32_t *duration)
{
  for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL)
  {
    // need to wait at least a single interval to prevent reading a jitter
    os_delay_us(DHT_TIMER_INTERVAL);
    if (GPIO_INPUT_GET(pin) == expected_pin_state)
    {
      if (duration)
      {
        *duration = i;
      }
      return true;
    }
  }

  return false;
}

static inline bool pollDHTCb(int pin, bool bits[DHT_DATA_BITS])
{
  uint32_t low_duration;
  uint32_t high_duration;

  // set the output to hight to have the high-low change for wakeup
  // not needed really, there is pullup resistor
  GPIO_OUTPUT_SET(pin, 1);
  os_delay_us(20 * 1000);

  // DHT wakeup signal Hold low for 20ms (datasheet at least 18ms)
  GPIO_OUTPUT_SET(pin, 0);
  os_delay_us(20 * 1000);

  // disable output function to receive low response from DHT
  GPIO_DIS_OUTPUT(pin);

  if (!dht_await_pin_state(pin, 40, false, NULL))
  {
    debug("DHT not response with low after wakeup.\n");
    return false;
  }

  // Step through Phase 'C', 88us (datasheet 80us)
  if (!dht_await_pin_state(pin, 88, true, NULL))
  {
    debug("DHT not response with high during initialiazation.\n");
    return false;
  }

  // Step through Phase 'D', 88us (datasheet 80us)
  if (!dht_await_pin_state(pin, 88, false, NULL))
  {
    debug("DHT not response with low to end initialization sequence.\n");
    return false;
  }

  // Read in each of the 40 bits of data...
  for (int i = 0; i < DHT_DATA_BITS; i++)
  {
    // 
    if (!dht_await_pin_state(pin, 65, true, &low_duration))
    {
      debug("DHT not response high to transmit of 1bit\n");
      return false;
    }
    if (!dht_await_pin_state(pin, 75, false, &high_duration))
    {
      debug("DHT not response low to finish transmit 1bit and start next cycle\n");
      return false;
    }
    // to start transmit 1bit DHT response low for 50us
    // bit 0 is identified by high for 26-28us
    // bit 1 is identified by high for 70us 
    bits[i] = high_duration > low_duration;
  }

  return true;
}

static inline int16_t dht_convert_data(enum sensor_type st, uint8_t msb, uint8_t lsb)
{
  int16_t data;

  if (st == SENSOR_DHT22)
  {
    data = msb & 0x7F;
    data <<= 8;
    data |= lsb;
    if (msb & BIT(7))
    {
      data = 0 - data; // convert it to negative
    }
  }
  else
  {
    data = msb * 10;
  }

  return data;
}

bool readDHT(enum sensor_type st, uint8 pin, int16_t *humidity, int16_t *temperature)
{
  bool bits[DHT_DATA_BITS];
  uint8_t data[DHT_DATA_BITS / 8] = {0};
  bool result;

  taskENTER_CRITICAL();
  result = pollDHTCb(pin, bits);
  taskEXIT_CRITICAL();

  if (!result)
  {
    return false;
  }

  for (uint8_t i = 0; i < DHT_DATA_BITS; i++)
  {
    // Read each bit into 'result' byte array...
    data[i / 8] <<= 1;
    data[i / 8] |= bits[i];
  }

  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
  {
    debug("Checksum failed, invalid data received from sensor\n");
    return false;
  }

  *humidity = dht_convert_data(st, data[0], data[1]);
  *temperature = dht_convert_data(st, data[2], data[3]);

  debug("Sensor data: humidity=%d, temp=%d\n", *humidity, *temperature);

  return true;
}
