#include "i2c.h"

static gpio_num_t i2c_gpio_sda = 4;
static gpio_num_t i2c_gpio_scl = 5;

static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

static i2c_master_bus_handle_t i2c_master_driver_initialize(void)
{
  i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_port,
    .scl_io_num = i2c_gpio_scl,
    .sda_io_num = i2c_gpio_sda,
    .glitch_ignore_cnt = 7,        
    .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));   
  return bus_handle;
}

int i2c_detect()
{  
  i2c_master_bus_handle_t bus = i2c_master_driver_initialize();
  uint8_t address;
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  for (int i = 0; i < 128; i += 16)
  {
    printf("%02x: ", i);
    for (int j = 0; j < 16; j++)
    {
      fflush(stdout);
      address = i + j;      
      esp_err_t ret = i2c_master_probe(bus, address, 20);
      if (ret == ESP_OK)
      {
        printf("%02x ", address);
      }
      else if (ret == ESP_ERR_TIMEOUT)
      {
        printf("UU ");
      }
      else
      {
        printf("-- ");
      }
    }
    printf("\r\n");
  }

  //ESP_ERROR_CHECK(i2c_master_bus_reset(bus)); 
  i2c_del_master_bus(bus);
  return 0;
}