#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"

enum {
  // Pins
  SPI_INT_PIN = 20,
  SPI_RX_PIN =  16,
  SPI_SCK_PIN = 18,
  SPI_TX_PIN =  19,
  SPI_CSN_PIN = 17,
  IMU_CLKIN_PIN = 21,
  DEBUG_PIN1 = 12, // This is also uart0 TX.
  DEBUG_PIN2 = 13, // This is also uart0 RX.

  // Registers (some defined for clarity).
  PWR_MGMT0     = 0x4E,
  ACCEL_DATA_X1 = 0x1F,
  ACCEL_DATA_X0 = 0x20,
  ACCEL_DATA_Y1 = 0x21,
  ACCEL_DATA_Y0 = 0x22,
  ACCEL_DATA_Z1 = 0x23,
  ACCEL_DATA_Z0 = 0x24,
  GYRO_DATA_X1  = 0x25,
  GYRO_DATA_X0  = 0x26,
  GYRO_DATA_Y1  = 0x27,
  GYRO_DATA_Y0  = 0x28,
  GYRO_DATA_Z1  = 0x29,
  GYRO_DATA_Z0  = 0x2A,
  INT_CONFIG1   = 0x64,
  INT_SOURCE0   = 0x65,
  INT_CONFIG    = 0x14,
  INTF_CONFIG1  = 0x4d,
  REG_BANK_SEL  = 0x76,
  INTF_CONFIG5  = 0x7b, // Note, this is in bank 1 , not bank 0.
};

int main() {
  // Init debug tools.
  if (true) {
    stdio_uart_init_full(uart0, 115200, DEBUG_PIN1, DEBUG_PIN2);
  }
  else {
    gpio_init(DEBUG_PIN1);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_disable_pulls(DEBUG_PIN1);
    gpio_put(DEBUG_PIN1, 1);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_disable_pulls(DEBUG_PIN2);
    gpio_put(DEBUG_PIN2, 1);
  }

  // SPI interupt pin initialization.
  gpio_init(SPI_INT_PIN);
  gpio_set_dir(SPI_INT_PIN, GPIO_IN);
  gpio_disable_pulls(SPI_INT_PIN);
  gpio_pull_up(SPI_INT_PIN);
  
  // SPI bus initialization. (1MHZ)(0b1 = high/low?)(phase = ??)(most sig. bit first)
  spi_init(spi0, 1000 * 1000);
  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI); 
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI); 
  gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI); 
  gpio_set_function(SPI_CSN_PIN, GPIO_FUNC_SPI); 

  // IMU CLKIN PWM pin initialization.
  const int kDivisorInt = 19; // No idea how (int, frac) clock divider works.
  const int kDivisorFrac = 9; // No idea how (int, frac) clock divider works.
  const int kWrapVal = 195;
  gpio_set_function(IMU_CLKIN_PIN, GPIO_FUNC_PWM);
  int slice_num = pwm_gpio_to_slice_num(IMU_CLKIN_PIN);
  pwm_set_wrap(slice_num, kWrapVal - 1);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, kWrapVal >> 1);
  pwm_set_clkdiv_int_frac(slice_num, kDivisorInt, kDivisorFrac);
  pwm_set_enabled(slice_num, true); // Square wave with frequency ~32.77kHz.
  
  // Create the IMU instruction buffer, out_buf, and the register buffer, in_buf
  const int kBufLen = 13;
  uint8_t out_buf[kBufLen], in_buf[kBufLen];

  // Do some one-time SPI writes.
  out_buf[0] = INT_CONFIG1; // Initialize interrupts.
  out_buf[1] = 0b00000000;
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = INT_SOURCE0;
  out_buf[1] = 0b00001000; // Change interrupt output from "Reset done" to "UI data ready".
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = INT_CONFIG;
  out_buf[1] = 0b00000010; // Set dataReady interrupt to push-pull. 
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = PWR_MGMT0;
  out_buf[1] = 0b00001111; // Place gyro and accel in low noise mode.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = INTF_CONFIG1;
  out_buf[1] = 0b10010101; // RTC clock input is required.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);

  out_buf[0] = REG_BANK_SEL;
  out_buf[1] = 0b00000001; // Change from bank 0 to bank 1.
  out_buf[2] = INTF_CONFIG5;
  out_buf[3] = 0b00000100; // Sets pin 9 function to CLKIN
  out_buf[4] = REG_BANK_SEL;
  out_buf[5] = 0b00000000; // Change from bank 1 to bank 0.
  spi_write_read_blocking(spi0, out_buf, in_buf, 6);

  // Clear the instruction buffer for safety reasons, then
  // prepare a read instruction that outputs concurrent registers (IMU data).
  for (int i = 0; i < kBufLen; i++) {
    out_buf[i] = 0;
  }
  out_buf[0] = (1 << 7) | ACCEL_DATA_X1;

  uint8_t count = 0x00; // This number is transmitted and used for data loss checking (sequence checking).
  while (true) {
    // Wait until the IMU signals new data on the SPI_INT_PIN.
    while(gpio_get(SPI_INT_PIN) == false) {}

    // Interrupt detected.
    // Increment count (00-ff).
    if (count == 0xff)
      count = 0x00;
    else
      count++;

    // Get all IMU data and send to in_buf. The information is stored from index 1 to index kBufLef-1.
    spi_write_read_blocking(spi0, out_buf, in_buf, kBufLen);

    // Print IMU data (debug feature).
    for (int i = 0; i < 12; i++)
      printf("%02x ", in_buf[i+1]);
    printf("\n");
  }
}
