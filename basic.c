/*!
 *  @brief Example shows basic setup of sensor which includes following
 *      Initialization of the interface.
 *      performing the sensor initialization.
 */

#include "stdio.h"
#include "bmp280.h"

#include <unistd.h> // for usleep
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

void sleep_ms(int milliseconds) // cross-platform sleep function
{
  usleep(milliseconds * 1000);
}

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

int file;

int main(void)
{
  // Create I2C bus
  // int file;
  char *bus = "/dev/i2c-2";
  if ((file = open(bus, O_RDWR)) < 0)
  {
    perror("Failed to open the bus. \n");
    exit(1);
  }
  // Get I2C device, BMP280 I2C address is 0x76(108)
  if(ioctl(file, I2C_SLAVE, 0x76)){
    printf("Failed to acquire bus access and/or talk to slave.\n");
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

// https://elinux.org/Interfacing_with_I2C_Devices

  int8_t rslt;
  struct bmp280_dev bmp;

  /* Map the delay function pointer with the function responsible for implementing the delay */
  bmp.delay_ms = delay_ms;

  /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
  bmp.dev_id = BMP280_I2C_ADDR_PRIM;

  /* Select the interface mode as I2C */
  bmp.intf = BMP280_I2C_INTF;

  /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
  bmp.read = i2c_reg_read;
  bmp.write = i2c_reg_write;

  /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

  // bmp.dev_id = 0;
  // bmp.read = spi_reg_read;
  // bmp.write = spi_reg_write;
  // bmp.intf = BMP280_SPI_INTF;

  rslt = bmp280_init(&bmp);
  printf("pase la inicializacion\n");
  print_rslt(" bmp280_init status", rslt);

  return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
  /* Implement the delay routine according to the target machine */
  sleep_ms(period_ms);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  /* Implement the I2C write routine according to the target machine. */
  printf("Voy a escribir\n");
  char adress[1] = {reg_addr};
  char content[length];

  for(int i=0; i<length; i++){
    content[i]=reg_data[i];
    printf("content[%d] %x\n", i, content[0]);
  }

  char* total = malloc((1+length) * sizeof(char)); // array to hold the result
  memcpy(total, adress, 1 * sizeof(char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total + 1 * sizeof(char), content, length * sizeof(char)); // copy 4 floats from y to total[4]...total[7]

  for(int i=0; i<length+1; i++){
    printf("total[%d] %x\n", i, total[i]);
  }

  int writeCount = write(file, total, length+1);

  free(total);
  return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */


int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  printf("Voy a leer\n");
  /* Implement the I2C read routine according to the target machine. */  
  int writeCount = write(file, &reg_addr, 1);

  if (read(file, reg_data, length) != length)
  {
    printf("Error : Input/output Error \n");
    exit(1);
    return -1;
  }
  
  return 0;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

  /* Implement the SPI write routine according to the target machine. */
  return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

  /* Implement the SPI read routine according to the target machine. */
  return -1;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
  if (rslt != BMP280_OK)
  {
    printf("%s\t", api_name);
    if (rslt == BMP280_E_NULL_PTR)
    {
      printf("Error [%d] : Null pointer error\r\n", rslt);
    }
    else if (rslt == BMP280_E_COMM_FAIL)
    {
      printf("Error [%d] : Bus communication failed\r\n", rslt);
    }
    else if (rslt == BMP280_E_IMPLAUS_TEMP)
    {
      printf("Error [%d] : Invalid Temperature\r\n", rslt);
    }
    else if (rslt == BMP280_E_DEV_NOT_FOUND)
    {
      printf("Error [%d] : Device not found\r\n", rslt);
    }
    else
    {
      /* For more error codes refer "*_defs.h" */
      printf("Error [%d] : Unknown error code\r\n", rslt);
    }
  }
}
