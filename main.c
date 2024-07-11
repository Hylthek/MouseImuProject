#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/util/queue.h"

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

// Pins
enum {
  // RP2040 GPIO pins (no UART control).
  kSpiIntPin = 20, // Goes low at IMU data ready.
  kSpiCsnPin = 17, // Chip select.
  kSpiSckPin = 18, // SPI clock
  kSpiTxPin =  19, // MOSI
  kSpiRxPin =  16, // MISO
  kImuClkinPin = 21, // Insert a 32.77kHz square wave to replace embedded IMU clock.
  kUartDePin  = 3, // used to disable rs485 driver #1.
  kUartDePin2 = 6, // used to disable rs485 driver #2.
  kUartRePin  = 2, // For both RS485 drivers.

  // Debugging pins.
  kDebugPin1 = 12,
  kDebugPin2 = 13,
  kLedPin = 25,
};

// IMU register addresses.
enum {
  // Note that some are defined only for reference as bulk reads exists).
  kPwrMgmt0    = 0x4E,
  kAccelDataX1 = 0x1F,
  kAccelDataX0 = 0x20,
  kAccelDataY1 = 0x21,
  kAccelDataY0 = 0x22,
  kAccelDataZ1 = 0x23,
  kAccelDataZ0 = 0x24,
  kGyroDataX1  = 0x25,
  kGyroDataX0  = 0x26,
  kGyroDataY1  = 0x27,
  kGyroDataY0  = 0x28,
  kGyroDataZ1  = 0x29,
  kGyroDataZ0  = 0x2A,
  kIntConfig1  = 0x64,
  kIntSource0  = 0x65,
  kIntConfig   = 0x14,
  kIntfConfig1 = 0x4d,
  kRegBankSel  = 0x76,
  kIntfConfig5 = 0x7b, // Note, this is in bank 1 , not bank 0.
};

// Constants
enum {
  kFixedToInt = 8, // The number of right shifts needed to turn the fixed point into its integer value or vise versa.
  kPrintfQueueMaxSize = 100000, // Designed to almost fill up heap memory ("OUT OF MEMORY" error occurs if too big).
};

// Configurables
enum {
  kLpfQueueMaxSize     = 10,  // The period (in samples) of the pseudo low pass filter used in the motion detector.
  kMovementThreshold   = 20,  // The required difference (in IMUunits) between two points to signal movement.
  kSteadinessThreshold = 25,  // The required time (in samples) of consistent inactivity to signal steadiness.
  kBiasThresholdLog2  = 10,   // The log2 of the number of samples used to generate a set of final biases (eg. 10 = 1.024 seconds of biasing).
  kHighpassCoefAccel = 99,    // When /100,  is the constant k in the highpass filter equation (0 => 0, 100 => out=in).
  kHighpassCoefVel   = 99,    // When /100, the constant k in the highpass filter equation (0 => 0, 100 => out=in).
  kLsbToDegrees = 170,        // When /100, the amount we divide IMU gyroscope samples by to get degrees per second.
};

// Globals
typedef int fixed_point_t;
fixed_point_t gDX = 0; // Fixed point numbers "added to" by main() "subtracted from" by send_hid_report(). 
fixed_point_t gDY = 0; // Fixed point numbers "added to" by main() "subtracted from" by send_hid_report().

// Function declarations.
void SpiInit();
void hid_task(void);
void LedTask(bool start_blink);
void WtoQ(float w[3], float q[4]);
void QuaternionMultiplication(float q_l[4], float q_r[4], float q_result[4]);
void MultiplyQuaternionByVector(float q[4], float v[3], float result[3]);

int main(void) {
  // Init debug printfs.
  if (true) {
    stdio_uart_init_full(uart0, 115200, kDebugPin1, kDebugPin2);
    gpio_init(kDebugPin2);
    gpio_set_dir(kDebugPin2, GPIO_OUT);
    gpio_put(kDebugPin2, 0);
  }
  else {
    gpio_init(kDebugPin1);
    gpio_init(kDebugPin2);
    gpio_set_dir(kDebugPin1, GPIO_OUT);
    gpio_set_dir(kDebugPin2, GPIO_OUT);
    gpio_put(kDebugPin1, 0);
    gpio_put(kDebugPin2, 0);
  }

  // Init and blink LED.
  { 
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);
    gpio_put(kLedPin, 0);
    bool led_on = 1;
    for (int i = 0; i < 10; i++) {
      gpio_put(kLedPin, led_on);
      led_on = !led_on;
      sleep_ms(100);
    }
  }

  // SPIbus initialization (also disables UART).
  SpiInit();

  // Create the IMU instruction buffer, out_buf, and the register buffer, in_buf
  // Prepare a read instruction that outputs concurrent registers (IMU data).
  const int kSpiBufLen = 13;
  uint8_t spi_outbuf[kSpiBufLen], spi_inbuf[kSpiBufLen];
  spi_outbuf[0] = (1 << 7) | kAccelDataX1;

  // Setup queues
  typedef struct {int16_t ax, ay, az, gx, gy, gz;} ImuSample; // Formatting a struct def this way puts it on the global namespace.
  queue_t lpf_queue; // A queue containing the last "kLpfPeriod" of IMU samples.
  queue_t printf_queue; // A queue that stores IMU data so it can be printed later asynchronously
  {
    queue_init(&lpf_queue, sizeof(ImuSample), kLpfQueueMaxSize);
    queue_init(&printf_queue, sizeof(int16_t), kPrintfQueueMaxSize);
  }

  // Init tinyusb.
  tusb_init();

  // Main loop.
  while (true) {
    // Routine tasks.
    {
      // tud_task() must be run consistently, main loop cannot stall, it should continue if anything.
      // Routine LED check (arg. is false). 
      // Poll for USB sending conditions (time, bus readiness, etc.).
      tud_task();
      LedTask(false);
      hid_task();
    }
    
    // IMU interrupt checking.
    if (gpio_get(kSpiIntPin) == true)
      continue;
    
    // Debug feature, pulse debug pin on interrupt negedge.
    gpio_put(kDebugPin2, 0);

    // Get all IMU data and send to spi_inbuf. The information is stored from index 1 to index 12.
    ImuSample sample_rxed;
    {
      // Convert all MEMS data uint8_ts into int16_ts.
      spi_write_read_blocking(spi0, spi_outbuf, spi_inbuf, kSpiBufLen);
      sample_rxed.ax = (spi_inbuf[ 1]<<8) + spi_inbuf[ 2];
      sample_rxed.ay = (spi_inbuf[ 3]<<8) + spi_inbuf[ 4];
      sample_rxed.az = (spi_inbuf[ 5]<<8) + spi_inbuf[ 6];
      sample_rxed.gx = (spi_inbuf[ 7]<<8) + spi_inbuf[ 8];
      sample_rxed.gy = (spi_inbuf[ 9]<<8) + spi_inbuf[10];
      sample_rxed.gz = (spi_inbuf[11]<<8) + spi_inbuf[12];
    }

    // Debug feature, print matlab data asynchronously.
    { 
      //int curr_time = board_millis();
      //queue_try_add(&printf_queue, &sample_rxed.ax);
      //queue_try_add(&printf_queue, &sample_rxed.ay);
      //queue_try_add(&printf_queue, &sample_rxed.az);
      //queue_try_add(&printf_queue, &sample_rxed.gx);
      //queue_try_add(&printf_queue, &sample_rxed.gy);
      //queue_try_add(&printf_queue, &sample_rxed.gz);
      //queue_try_add(&printf_queue, &curr_time);
      //if (queue_is_full(&printf_queue)) {
      //  int i = 1;
      //  while (!queue_is_empty(&printf_queue)) {
      //    int16_t out;
      //    queue_try_remove(&printf_queue, &out);
      //    printf("%d ", out);
      //    if (i == 7) {
      //      printf("\n");
      //      i = 1;
      //    }
      //    else {
      //      i++;
      //    }
      //  }
      //  return 0;
      //} 
    }

    // Movement detector.
    bool is_moving = false;
    {
      // Only processes data if there are enough samples.
      // Unconditionally add sample to LPF queue at end.
      if (queue_is_full(&lpf_queue)) {
        ImuSample sample_popped;
        queue_try_remove(&lpf_queue, &sample_popped);
  
        // Now sample_rxed and sample_popped are "kLpfPeriod" samples apart from each other.
        // Calculate all 6 differences.
        int16_t diff[6] = {0};
        diff[0] = abs(sample_rxed.ax - sample_popped.ax);
        diff[1] = abs(sample_rxed.ay - sample_popped.ay);
        diff[2] = abs(sample_rxed.az - sample_popped.az);
        diff[3] = abs(sample_rxed.gx - sample_popped.gx);
        diff[4] = abs(sample_rxed.gy - sample_popped.gy);
        diff[5] = abs(sample_rxed.gz - sample_popped.gz);
  
        // Calculate the maximum difference between the 6.
        int16_t max_diff = 0;
        for (int i = 0; i < 6; i++)
          max_diff = (max_diff > diff[i]) ? (max_diff) : (diff[i]);
  
        // Update logic.
        static int steadiness_counter = 0; // A counter value that starts at kSteadinessThreshold and counts down.
        if (max_diff > kMovementThreshold) {
          // Set moving and reset counter.
          is_moving = true;
          steadiness_counter = kSteadinessThreshold;
  
        }
        else {
          // Check if last movement was still too recent.
          if (steadiness_counter > 0)
            steadiness_counter--;
          else
            is_moving = false;
        }
      }
      queue_try_add(&lpf_queue, &sample_rxed);
    }

    // Debug feature, write movement boolean to LED.
    {
      if (is_moving)
        LedTask(true);
    }

    // Biasing.
    bool is_biased = false;
    static fixed_point_t bias_final_ax = 0, bias_final_ay = 0, bias_final_az = 0, // Holds useable bias values.
                         bias_final_gx = 0, bias_final_gy = 0, bias_final_gz = 0; // Note, acceleration biases are indistinguishable from gravity.
    {
      // If not moving, add to biasing average.
      // If moving, reset biasing average.
      static long int bias_avg_ax = 0, bias_avg_ay = 0, bias_avg_az = 0,
                      bias_avg_gx = 0, bias_avg_gy = 0, bias_avg_gz = 0; // Holds incompletely averaged bias values.
      static int num_samples_averaged = 0; // Holds how many samples are contributing to the current average.
      if (!is_moving) {
        bias_avg_ax += sample_rxed.ax; // These MOST LIKELY wont overflow.
        bias_avg_ay += sample_rxed.ay; // We will later shift the bias_avg values to get bias_final.
        bias_avg_az += sample_rxed.az; 
        bias_avg_gx += sample_rxed.gx; 
        bias_avg_gy += sample_rxed.gy; 
        bias_avg_gz += sample_rxed.gz; 
        num_samples_averaged++;
  
        // If there are now enough samples, update bias_final and reset average.
        const uint kBiasThreshold = 0b1 << kBiasThresholdLog2;
        if (num_samples_averaged >= kBiasThreshold) {
          is_biased = true;
           
          bias_final_ax = bias_avg_ax << kFixedToInt >> kBiasThresholdLog2; // Reminder1: bias_final is a fixed_point_t,
          bias_final_ay = bias_avg_ay << kFixedToInt >> kBiasThresholdLog2; // this is the shifting alluded to earlier.
          bias_final_az = bias_avg_az << kFixedToInt >> kBiasThresholdLog2; // Reminder2: bitshift associativity is left to right.
          bias_final_gx = bias_avg_gx << kFixedToInt >> kBiasThresholdLog2; 
          bias_final_gy = bias_avg_gy << kFixedToInt >> kBiasThresholdLog2; // Second bit shift is analogous to the usual dividing by num_samples_averaged.
          bias_final_gz = bias_avg_gz << kFixedToInt >> kBiasThresholdLog2;
  
          // Debug feature, blink LED when biases are calculated.
          LedTask(true);
  
          // Reset bias averaging.
          bias_avg_ax = 0; bias_avg_ay = 0; bias_avg_az = 0;
          bias_avg_gx = 0; bias_avg_gy = 0; bias_avg_gz = 0;
          num_samples_averaged = 0;
        }
      }
      else {
        bias_avg_ax = 0; bias_avg_ay = 0; bias_avg_az = 0;
        bias_avg_gx = 0; bias_avg_gy = 0; bias_avg_gz = 0;
        num_samples_averaged = 0;
      }
    }

    // Update orientation quaternion.
    static float orientation_quat[4] = {1, 0, 0, 0}; // Integrated rotation speed, represented as a quaternion.
    {
      if (is_biased) {
        orientation_quat[0] = 1;
        orientation_quat[1] = 0;
        orientation_quat[2] = 0;
        orientation_quat[3] = 0;
      }
      
      // Turn net rotation into a quaternion.
      const float k = kLsbToDegrees/10.f;
      float sample_rotation_vec[3] = {
        (sample_rxed.gx - (bias_final_gx/256.f)) / k * M_PI / 180.f * 0.001f, // Units are in radians.
        (sample_rxed.gy - (bias_final_gy/256.f)) / k * M_PI / 180.f * 0.001f, // Units are in radians.
        (sample_rxed.gz - (bias_final_gz/256.f)) / k * M_PI / 180.f * 0.001f  // Units are in radians.
      };
      float sample_rot_quat[4] = {0};
      WtoQ(sample_rotation_vec, sample_rot_quat);
  
      // Update orientation_quat.
      // We need to create a copy because QuaternionMultiplication references itself.
      float orientation_quat_cpy[4] = {orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3]};
      QuaternionMultiplication(orientation_quat_cpy, sample_rot_quat, orientation_quat);
    }

    // Calculate orientation vector.
    float orientation_vec[3];
    {
      float unit_z[3] = {0,0,1};
      MultiplyQuaternionByVector(orientation_quat, unit_z, orientation_vec);
    }

    // Calculate global_accel.
    float global_accel_vec[3];
    {
      float sample_accel_vec[3] = {sample_rxed.ax, sample_rxed.ay, sample_rxed.az};
      MultiplyQuaternionByVector(orientation_quat, sample_accel_vec, global_accel_vec); 
      global_accel_vec[0] -= bias_final_ax/256.f;
      global_accel_vec[1] -= bias_final_ay/256.f;
      global_accel_vec[2] -= bias_final_az/256.f;
    }

    // Calculate highpass_accel_vec.
    static float highpass_accel_vec[3] = {0};
    {
      static float prev_sample[3] = {0};
      float k;
      if (is_moving)
        k = 1;
      else
        k = kHighpassCoefAccel/100.f;

      // Calculate the change in acceleration.
      float diff[3];
      for (int i = 0; i < 3; i++)
        diff[i] = global_accel_vec[i]-prev_sample[i];

      // Update highpass_accel_vec.
      for (int i = 0; i < 3; i++)
        highpass_accel_vec[i] = k*(highpass_accel_vec[i] + diff[i]);

      // Update prev sample.
      for (int i = 0; i < 3; i++)
        prev_sample[i] = global_accel_vec[i];
    }

    // Update velocity integral.
    static float global_vel_vec[3] = {0};
    {
        
      for (int i = 0; i < 3; i++) {
        if (is_biased)
          global_vel_vec[i] = 0;
        else
          global_vel_vec[i] += highpass_accel_vec[i]/1000.f;
      }
    }

    // Calculate highpass_vel_vec.
    static float highpass_vel_vec[3] = {0};
    {
      static float prev_sample[3] = {0};
      float k;

      if (is_moving)
        k = 1.f;
      else
        k = kHighpassCoefVel/100.f;

      // Calculate the change in velocity.
      float diff[3];
      for (int i = 0; i < 3; i++)
        diff[i] = global_vel_vec[i]-prev_sample[i];

      // Update highpass_vel_vec.
      for (int i = 0; i < 3; i++)
        highpass_vel_vec[i] = k*(highpass_vel_vec[i] + diff[i]);

      // Update prev sample.
      for (int i = 0; i < 3; i++)
        prev_sample[i] = global_vel_vec[i];
    }

    // Debug feature, print values intermittently.
    {
      static int count = 0;
      if (count == 1000) {
        count = 0;
        //printf("\n%10f %10f %10f\n", global_accel_vec[0], global_accel_vec[1], global_accel_vec[2]);
        //printf("%f\n\t%f\n\t%f\n\t%f\n",
        //  acosf(orientation_quat[0])*2*180/M_PI,
        //  orientation_quat[1]/sinf(acosf(orientation_quat[0])),
        //  orientation_quat[2]/sinf(acosf(orientation_quat[0])),
        //  orientation_quat[3]/sinf(acosf(orientation_quat[0]))
        //);
        //printf("%3.2f %3.2f %3.2f\n", orientation_vec[0], orientation_vec[1], orientation_vec[2]);
        printf("%3.2f %3.2f %3.2f\n", highpass_accel_vec[0], highpass_accel_vec[1], highpass_accel_vec[2]);
        printf("%3.2f %3.2f %3.2f\n", highpass_vel_vec[0], highpass_vel_vec[1], highpass_vel_vec[2]);
      }
      else {
        count++;
      }
    }

    // Change mouse movement globals (communicates with hid_task()).
    { 
      gDX -= highpass_vel_vec[0] * 9000; // Positive = right.
      gDY += highpass_vel_vec[1] * 9000; // Positive = down.
      //gDX -= highpass_accel_vec[0] * 750; // Positive = right.
      //gDY += highpass_accel_vec[1] * 750; // Positive = down.
    }

    // Debug feature, unpulse at end of interrupt.
    gpio_put(kDebugPin2, 1);
  }
}

// USB HID send.
static void send_hid_report(uint8_t report_id) {
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;

  // Calculate amount to move cursor / "subtract" from gDX & gDY.
  // turn to ints and manually saturate.
  const int kShiftFactor = 18; // To shift output for useability.
  int dx = gDX >> kShiftFactor;
  int dy = gDY >> kShiftFactor;
  if (dx > 127)
    dx = 127;
  if (dx < -128)
    dx = -128;
  if (dy > 127)
    dy = 127;
  if (dy < -128)
    dy = -128;

  // Debug feature, print mouse report deltas.
  //printf("%02x %02x\n", (uint8_t)dx, (uint8_t)dy);

  // Move mouse.
  tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, dx, dy, 0, 0);

  // Subtract the amount moved.
  gDX -= dx << kShiftFactor;
  gDY -= dy << kShiftFactor;
}

// Invoked when device is mounted.
void tud_mount_cb(void) {}

// Invoked when device is unmounted.
void tud_umount_cb(void) {}

// Invoked when usb bus is suspended.
// remote_wakeup_en : if host allows us to perform remote wakeup.
// Within 7ms, device must draw an average of current less than 2.5 mA from bus.
void tud_suspend_cb(bool remote_wakeup_en) {}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {}

// Calls send_hid_report if it has been long nough since the last call.
void hid_task(void) {
  // Poll every 10ms.
  const uint32_t interval_ms = 1;
  static uint32_t start_ms = 0;
  if ( board_millis() - start_ms < interval_ms)
    return; // not enough time
  start_ms += interval_ms;

  // Remote wakeup.
  if (tud_suspended()) {
    // Wake up host if we are in suspend mode.
    // and REMOTE_WAKEUP feature is enabled by host.
    tud_remote_wakeup();
  }
  else {
    // Send the report.
    send_hid_report(REPORT_ID_MOUSE);
  }
}

// Invoked when sent REPORT successfully to host.
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {}

// Invoked when received GET_REPORT control request. Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {return 0;}

// Invoked when received SET_REPORT control request or received data on OUT endpoint ( Report ID = 0, Type = 0 ).
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {}

// Call with true to initiaze a blink, call with false to turn off the LED if the blink is done. 
void LedTask(bool start_blink) {

  const int millis_on = 25;
  static int start_millis = 0;

  if (start_blink) {
    board_led_write(true);
    start_millis = board_millis();
  }

  if (board_millis() - start_millis >= millis_on) {
    board_led_write(false);
  }
}

// Initialize SPIbus and disable UARTbus.
void SpiInit() {
  // Disable UARTbus.
  gpio_init(kUartDePin);
  gpio_set_dir(kUartDePin, GPIO_OUT);
  gpio_disable_pulls(kUartDePin);
  gpio_put(kUartDePin, 0);
  gpio_init(kUartDePin2);
  gpio_set_dir(kUartDePin2, GPIO_OUT);
  gpio_disable_pulls(kUartDePin2);
  gpio_put(kUartDePin2, 0);
  gpio_init(kUartRePin);
  gpio_set_dir(kUartRePin, GPIO_OUT);
  gpio_disable_pulls(kUartRePin);
  gpio_put(kUartRePin, 1);

  // SPI interupt pin initialization.
  gpio_init(kSpiIntPin);
  gpio_set_dir(kSpiIntPin, GPIO_IN);
  gpio_disable_pulls(kSpiIntPin);
  gpio_pull_up(kSpiIntPin);
  
  // SPI bus initialization. (1MHZ)(0b1 = high/low?)(phase = ??)(most sig. bit first)
  spi_init(spi0, 1000 * 1000);
  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(kSpiRxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kSpiSckPin, GPIO_FUNC_SPI); 
  gpio_set_function(kSpiTxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kSpiCsnPin, GPIO_FUNC_SPI); 

  // IMU CLKIN PWM pin initialization.
  const int kDivisorInt = 19; // No idea how (int, frac) clock divider works.
  const int kDivisorFrac = 9; // No idea how (int, frac) clock divider works.
  const int kWrapVal = 195;
  gpio_set_function(kImuClkinPin, GPIO_FUNC_PWM);
  int slice_num = pwm_gpio_to_slice_num(kImuClkinPin);
  pwm_set_wrap(slice_num, kWrapVal - 1);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, kWrapVal >> 1);
  pwm_set_clkdiv_int_frac(slice_num, kDivisorInt, kDivisorFrac);
  pwm_set_enabled(slice_num, true); // Square wave with frequency ~32.77kHz.

  // Create the IMU instruction buffer, out_buf, and the register buffer, in_buf
  uint8_t out_buf[10], in_buf[10]; // 10 is an arbitrary size.
  
  // Do some one-time SPI writes.
  out_buf[0] = kIntConfig1; // Initialize interrupts.
  out_buf[1] = 0b00000000;
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = kIntSource0;
  out_buf[1] = 0b00001000; // Change interrupt output from "Reset done" to "UI data ready".
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = kIntConfig;
  out_buf[1] = 0b00000010; // Set dataReady interrupt to push-pull. 
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = kPwrMgmt0;
  out_buf[1] = 0b00001111; // Place gyro and accel in low noise mode.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = kIntfConfig1;
  out_buf[1] = 0b10010101; // RTC clock input is required.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);

  out_buf[0] = kRegBankSel;
  out_buf[1] = 0b00000001; // Change from bank 0 to bank 1.
  out_buf[2] = kIntfConfig5;
  out_buf[3] = 0b00000100; // Sets pin 9 function to CLKIN
  out_buf[4] = kRegBankSel;
  out_buf[5] = 0b00000000; // Change from bank 1 to bank 0.
  spi_write_read_blocking(spi0, out_buf, in_buf, 6);
}

// Sin(x)/x.
float Sinc(float x) {
  // If |x| < 1e-4, use taylor series expansion.
  // The approximation is accurate enough.
  if (fabs(x)  < 1.0e-4f)
    return 1.0f - x*x*0.16666666666666666666667f;
  else
    return sinf(x)/x;
}

// Turns a vector into a quaternion rotation where space is rotated |v| radians about v (Right-hand rule).
void WtoQ(float w[3], float q[4]) {
  float w_mag = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
  float theta_half = 0.5f * w_mag;
  float k = 0.5f * Sinc(theta_half);
  q[0] = cosf(theta_half);
  q[1] = k * w[0];
  q[2] = k * w[1];
  q[3] = k * w[2];
}

// Quaternion multiplication left-right
void QuaternionMultiplication(float q_l[4], float q_r[4], float q_result[4]) {
  q_result[0] = q_l[0]*q_r[0] - q_l[1]*q_r[1] - q_l[2]*q_r[2] - q_l[3]*q_r[3];
  q_result[1] = q_l[0]*q_r[1] + q_l[1]*q_r[0] + q_l[2]*q_r[3] - q_l[3]*q_r[2];
  q_result[2] = q_l[0]*q_r[2] - q_l[1]*q_r[3] + q_l[2]*q_r[0] + q_l[3]*q_r[1];
  q_result[3] = q_l[0]*q_r[3] + q_l[1]*q_r[2] - q_l[2]*q_r[1] + q_l[3]*q_r[0];
}

// Vector cross product.
void CrossProduct(float a[3], float b[3], float c[3]) {
  a[0] = (b[1]*c[2] - b[2]*c[1]);
  a[1] = (b[2]*c[0] - b[0]*c[2]);
  a[2] = (b[0]*c[1] - b[1]*c[0]);
}

// Equivalent to q*v*conj(q) where v is a quaternion with real part = 0 and i,j,k = vector's(x,y,z).
void MultiplyQuaternionByVector(float q[4], float v[3], float result[3]) {
  float w = q[0];
  float *vec = q+1; // vec[3] = {q[1], q[2], q[3]}
  float uv[3], vecuv[3];
  CrossProduct(uv,vec,v);
  CrossProduct(vecuv, vec, uv);
  result[0] = v[0] + 2*(w*uv[0] + vecuv[0]);
  result[1] = v[1] + 2*(w*uv[1] + vecuv[1]);
  result[2] = v[2] + 2*(w*uv[2] + vecuv[2]);
}
