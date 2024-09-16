#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/util/queue.h"

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

// Pins
enum {
  // RP2040 GPIO pins (no UART control).
  kImuRxPin =  16, // MISO
  kImuCsnPin = 17, // Chip select.
  kImuSckPin = 18, // SPI clock
  kImuTxPin =  19, // MOSI
  kImuIntPin = 20, // Goes low at IMU data ready.
  kImuClkinPin = 21, // Insert a 32.77kHz square wave to replace embedded IMU clock.
  kPixartIntPin = 7, // Negedge signals data ready.
  kPixartRxPin =  8,
  kPixartCsnPin = 9,
  kPixartSckPin = 10,
  kPixartTxPin =  11,
  kPixartResetnPin = 12,
  kTtlTxPin = 4, // For communicating with the other board (SE-UART).
  kTtlRxPin = 5, // For communicating with the other board (SE-UART).

  // Debugging pins.
  kDebugPin1 = 0,
  kDebugPin2 = 1,
  kLedPin = 25,
};

// Chip register addresses.
enum {
  // IMU registers
  // Note that some are defined only for reference as bulk reads exists).
  kPwrMgmt0     = 0x4E,
  kAccelDataX1  = 0x1F,
  kAccelDataX0  = 0x20,
  kAccelDataY1  = 0x21,
  kAccelDataY0  = 0x22,
  kAccelDataZ1  = 0x23,
  kAccelDataZ0  = 0x24,
  kGyroDataX1   = 0x25,
  kGyroDataX0   = 0x26,
  kGyroDataY1   = 0x27,
  kGyroDataY0   = 0x28,
  kGyroDataZ1   = 0x29,
  kGyroDataZ0   = 0x2A,
  kIntConfig1   = 0x64,
  kIntSource0   = 0x65,
  kIntConfig    = 0x14,
  kIntfConfig1  = 0x4d,
  kRegBankSel   = 0x76,
  kIntfConfig5  = 0x7b, // Note, this is in bank 1 , not bank 0.
  kAccelConfig0 = 0x50,
  kGyroConfig0  = 0x4f,

  // Pixart registers (confidential)
  kMotion = 0x02,
  kDeltaXL = 0x03,
  kDeltaYL = 0x04,
  kPowerUpReset = 0x3a,
};

// Constants
enum {
  kPrintfQueueMaxSize = 90000, // Designed to almost fill up heap memory ("OUT OF MEMORY" error occurs if too big).
  kByteQueueMaxSize = 100,
};

// Configurables
enum {
  kMovementLpfQueueMaxSize = 10,     // The period (in samples) of the pseudo low pass filter used in the motion detector.
  kMovementThreshold       = 80,     // The required difference (in IMUunits) between two points to signal movement.
  kSteadinessThreshold     = 25,     // The required time (in samples) of consistent inactivity to signal steadiness.
  kHoverLpfQueueMaxSize      = 100,  // The period (in samples) of the pseudo low pass filter used in the motion detector.
  kHoverThreshold            = 1000, // The required difference (in gyroscope units) between two (gx|gy) points to signal hovering.
  kGroundedThreshold         = 10,  // The required time (in samples) of consistent non-hovering to signal grounding.
  kBiasThreshold           = 1000,     // The number of samples used to generate a set of final biases.
  kHighpassCoefAccel         = 90,   // When /100,  is the constant k in the highpass filter equation (0 => 0, 100 => out=in).
  kHighpassCoefVel           = 10,   // When /100, the constant k in the highpass filter equation (0 => 0, 100 => out=in).
  kLsbPerDegrees           = 668,    // When /10, the amount we divide IMU gyroscope samples by to get degrees per second.
};
static const float kOrientationCorrectionFactor = 0.001;

// Globals
float gDXMouse = 0; // Numbers "added to" by main() "subtracted from" by send_hid_report(). 
float gDYMouse = 0; // Numbers "added to" by main() "subtracted from" by send_hid_report().
float gDXGamepad = 0; // Ditto, but for the gamepad HID subclass.
float gDYGamepad = 0; // Ditto, but for the gamepad HID subclass.
queue_t gByteQueue; // Pushed by Uart1InterruptHandler(), popped by main().
bool gMB1 = false, gMB2 = false, gMB3 = false; // Set in main() and used in send_hid_report().
int8_t gScrlUps = 0; // Set in main() and used in send_hid_report().
int16_t gHeadRoll = 0;  // Set in main() and used in send_hid_report(). Units are 1/10ths of a degree.
int16_t gHeadPitch = 0; // Set in main() and used in send_hid_report(). Units are 1/10ths of a degree.
int16_t gHeadYaw = 0;   // Set in main() and used in send_hid_report(). Units are 1/10ths of a degree.
int gMouseMode = 0; // 0 = normal mode, 1 = elliptical-orbit-demo mode.

// Function declarations.
void ImuInit();
void PixartInit();
void hid_task(void);
void LedTask(bool start_blink);
void WtoQ(float w[3], float q[4]);
void QuaternionMultiplication(float q_l[4], float q_r[4], float q_result[4]);
void MultiplyQuaternionByVector(float q[4], float v[3], float result[3]);
void RotationVectorAtoB(const float a[3], const float b[3], float result[3]);
float VectorLength(const float a[3]);
void RfromQ (float R[3][3], const float q[4]);
void HeadBoardReqTask();
void Uart1InterruptHandler();

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

  // IMU SPIbus initialization.
  // Pixart sensor SPIbus initialization.
  ImuInit();
  PixartInit();

  // Create the IMU instruction buffer, out_buf, and the register buffer, in_buf
  // Prepare a read instruction that outputs concurrent registers (IMU data).
  const int kImuSpiBufLen = 13;
  uint8_t imu_spi_outbuf[kImuSpiBufLen], imu_spi_inbuf[kImuSpiBufLen];
  imu_spi_outbuf[0] = (1 << 7) | kAccelDataX1;

  // Prepare a read instruction that outputs concurrent registers for optical Pixart data.
  const int kPixartSpiBufLen = 6;
  uint8_t pixart_spi_outbuf[kPixartSpiBufLen], pixart_spi_inbuf[kPixartSpiBufLen];
  pixart_spi_outbuf[0] = 0x02;
  pixart_spi_outbuf[2] = 0x03;
  pixart_spi_outbuf[4] = 0x04;

  // Setup TTL(UART) HeadBoard communication.
  uart_init(uart1, 1000*1000); // 1MHZ
  uart_set_format(uart1, 8, 1, UART_PARITY_NONE); // (8bit data)(1bit stop)
  gpio_set_function(kTtlTxPin, GPIO_FUNC_UART);
  gpio_set_function(kTtlRxPin, GPIO_FUNC_UART);
  uart_set_fifo_enabled(uart1, false);
  uart_set_hw_flow(uart1, false, false);

  // UART interrupt initialization
  irq_set_exclusive_handler(UART1_IRQ, Uart1InterruptHandler);
  irq_set_enabled(UART1_IRQ, true);

  // Enable the UART to send interrupts - RX and RT (BE too apparently).
  uart_set_irq_enables(uart1, true, false);

  // Setup queues
  typedef struct {int16_t ax, ay, az, gx, gy, gz;} ImuSample; // Formatting a struct def this way puts it on the global namespace.
  queue_t movement_lpf_queue; // A queue containing the last "kMovementLpfPeriod" of IMU samples.
  queue_t hover_lpf_queue; // A queue containing the last "kHoverLpfPeriod" of IMU samples.
  queue_t printf_queue; // A queue that stores IMU data so it can be printed later asynchronously (for Matlab analysis).
  {
    queue_init(&movement_lpf_queue, sizeof(ImuSample), kMovementLpfQueueMaxSize);
    queue_init(&hover_lpf_queue, sizeof(ImuSample), kHoverLpfQueueMaxSize);
    queue_init(&printf_queue, sizeof(int16_t), kPrintfQueueMaxSize);
    queue_init(&gByteQueue, sizeof(uint8_t), kByteQueueMaxSize);
  }

  // Init tinyusb.
  tusb_init();

  // Main loop.
  while (true) {
    // Routine tasks.
    {
      // tud_task() must be run consistently, main loop cannot stall, it should continue if anything.
      // Routine LED check (false = can only turn off LED). 
      // Poll for USB sending conditions (time, bus readiness, etc.).
      // Send a HeadBoard data request. 
      tud_task();
      LedTask(false);
      hid_task();
      HeadBoardReqTask();
    }
    
    // IMU interrupt checking.
    if (gpio_get(kImuIntPin) == false) {
      // Debug feature, pulse debug pin on interrupt negedge.
      gpio_put(kDebugPin2, 0);
  
      // Get all IMU data and send to imu_spi_inbuf. The information is stored from index 1 to index 12.
      ImuSample sample_rxed;
      {
        // Convert all MEMS data uint8_ts into int16_ts.
        spi_write_read_blocking(spi0, imu_spi_outbuf, imu_spi_inbuf, kImuSpiBufLen);
        sample_rxed.ax = (imu_spi_inbuf[ 1]<<8) + imu_spi_inbuf[ 2];
        sample_rxed.ay = (imu_spi_inbuf[ 3]<<8) + imu_spi_inbuf[ 4];
        sample_rxed.az = (imu_spi_inbuf[ 5]<<8) + imu_spi_inbuf[ 6];
        sample_rxed.gx = (imu_spi_inbuf[ 7]<<8) + imu_spi_inbuf[ 8];
        sample_rxed.gy = (imu_spi_inbuf[ 9]<<8) + imu_spi_inbuf[10];
        sample_rxed.gz = (imu_spi_inbuf[11]<<8) + imu_spi_inbuf[12];
      }
  
      // Debug feature, print matlab data asynchronously.
      if (false) { 
        queue_try_add(&printf_queue, &sample_rxed.ax);
        queue_try_add(&printf_queue, &sample_rxed.ay);
        queue_try_add(&printf_queue, &sample_rxed.az);
        queue_try_add(&printf_queue, &sample_rxed.gx);
        queue_try_add(&printf_queue, &sample_rxed.gy);
        queue_try_add(&printf_queue, &sample_rxed.gz);
        if (queue_is_full(&printf_queue)) {
          int i = 1;
          while (!queue_is_empty(&printf_queue)) {
            int16_t out;
            queue_try_remove(&printf_queue, &out);
            printf("%d ", out);
            if (i == 6) {
              printf("\n");
              i = 1;
            }
            else {
              i++;
            }
          }
          return 0;
        }
        continue; 
      }
  
      // Movement detector.
      bool is_moving = true;
      if (true) {
        // Only processes data if there are enough samples.
        // Unconditionally add sample to LPF queue at end.
        if (queue_is_full(&movement_lpf_queue)) {
          ImuSample sample_popped;
          queue_try_remove(&movement_lpf_queue, &sample_popped);
    
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
          // Also calculate the maximum difference between gx and gy.
          int16_t max_diff = 0;
          for (int i = 0; i < 6; i++)
            max_diff = (max_diff > diff[i]) ? (max_diff) : (diff[i]);
          int16_t max_diff_gxgy = 0;
          max_diff_gxgy = (max_diff_gxgy > diff[3]) ? (max_diff_gxgy) : (diff[3]);
          max_diff_gxgy = (max_diff_gxgy > diff[4]) ? (max_diff_gxgy) : (diff[4]);
    
          // Update logic (is_moving).
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
        queue_try_add(&movement_lpf_queue, &sample_rxed);
      }
  
      // Hover detector.
      bool is_hovering = true;
      if (false) {
        // Only processes data if there are enough samples.
        // Unconditionally add sample to LPF queue at end.
        if (queue_is_full(&hover_lpf_queue)) {
          ImuSample sample_popped;
          queue_try_remove(&hover_lpf_queue, &sample_popped);
    
          // Now sample_rxed and sample_popped are "kLpfPeriod" samples apart from each other.
          // Calculate all 6 differences.
          int16_t diff[3] = {0};
          diff[0] = abs(sample_rxed.gx - sample_popped.gx);
          diff[1] = abs(sample_rxed.gy - sample_popped.gy);
          diff[2] = abs(sample_rxed.az - sample_popped.az);
    
          // Calculate the maximum difference between gx and gy.
          int16_t max_diff = 0;
          for (int i = 0; i < 2; i++)
            max_diff = (max_diff > diff[i]) ? (max_diff) : (diff[i]);
    
          // Update logic (is_moving).
          static int grounded_counter = 0; // A counter value that starts at kSteadinessThreshold and counts down.
          if (max_diff > kHoverThreshold) {
            // Set moving and reset counter.
            is_hovering = true;
            grounded_counter = kGroundedThreshold;
    
          }
          else {
            // Check if last movement was still too recent.
            if (grounded_counter > 0)
              grounded_counter--;
            else
              is_hovering = false;
          }
        }
        queue_try_add(&hover_lpf_queue, &sample_rxed);
      }
  
      // Biasing.
      static bool is_biased = false;
      static float gravity_vec[3] = {0}; // Note, accelerometer bias is indistinguishable from gravity.
      static float gyro_bias_vec[3] = {0};
      if (!is_biased) {
        // If not moving, add to biasing average.
        // If moving, reset biasing average.
        static float bias_avg_ax = 0, bias_avg_ay = 0, bias_avg_az = 0,
                    bias_avg_gx = 0, bias_avg_gy = 0, bias_avg_gz = 0; // Holds incompletely averaged bias values.
        static int num_samples_averaged = 0; // Holds how many samples are contributing to the current average.
        if (!is_moving) {
          bias_avg_ax += sample_rxed.ax;
          bias_avg_ay += sample_rxed.ay;
          bias_avg_az += sample_rxed.az; 
          bias_avg_gx += sample_rxed.gx; 
          bias_avg_gy += sample_rxed.gy; 
          bias_avg_gz += sample_rxed.gz; 
          num_samples_averaged++;
    
          // If there are now enough samples, update bias_final and reset average.
          if (num_samples_averaged >= kBiasThreshold) {
            is_biased = true;
            
            gravity_vec[0] = bias_avg_ax / num_samples_averaged;
            gravity_vec[1] = bias_avg_ay / num_samples_averaged;
            gravity_vec[2] = bias_avg_az / num_samples_averaged;
            gyro_bias_vec[0] = bias_avg_gx / num_samples_averaged;
            gyro_bias_vec[1] = bias_avg_gy / num_samples_averaged;
            gyro_bias_vec[2] = bias_avg_gz / num_samples_averaged;
    
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
  
      // Update orientation quaternion with gyroscope values.
      static float orientation_quat[4] = {1, 0, 0, 0}; // Integrated rotation speed, represented as a quaternion.
      {
        // Turn net rotation into a quaternion.
        static const float k = 10.0/kLsbPerDegrees * M_PI / 180.0 / 1024.0;
        float sample_rotation_vec[3] = {
          (sample_rxed.gx - gyro_bias_vec[0]) * k, // Units are in radians.
          (sample_rxed.gy - gyro_bias_vec[1]) * k, // Units are in radians.
          (sample_rxed.gz - gyro_bias_vec[2]) * k  // Units are in radians.
        };
        float sample_rot_quat[4] = {0};
        WtoQ(sample_rotation_vec, sample_rot_quat);
    
        // Update orientation_quat.
        // We need to create a copy because QuaternionMultiplication references itself.
        float orientation_quat_cpy[4] = {orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3]};
        QuaternionMultiplication(orientation_quat_cpy, sample_rot_quat, orientation_quat);
  
        // Normalize orientation to eliminate numerical error.
        float quat_mag_sum = 0;
        for (int i = 0; i < 4; i++)
          quat_mag_sum += orientation_quat[i]*orientation_quat[i];
        float quat_mag = sqrtf(quat_mag_sum);
        for (int i = 0; i < 4; i++)
          orientation_quat[i] = orientation_quat[i]/quat_mag;
      }
  
      // Calculate orientation vector.
      float orientation_vec[3] = {0};
      if (false) {
        float unit_z[3] = {0,0,1};
        MultiplyQuaternionByVector(orientation_quat, unit_z, orientation_vec);
      }
  
      // Correct orientation_quat a percentage of the way.
      {
        static bool biased = false;
        if (is_biased)
          biased = true;
  
        if (biased) {
          float a_hat[3] = {sample_rxed.ax, sample_rxed.ay, sample_rxed.az};
          float a[3];
          MultiplyQuaternionByVector(orientation_quat, a_hat, a);
          const float a_mag = VectorLength(a);
          float a_norm[3] = {a[0]/a_mag, a[1]/a_mag, a[2]/a_mag};
  
          const float bias_mag = VectorLength(gravity_vec);
          float bias_vec_norm[3] = {gravity_vec[0]/bias_mag, gravity_vec[1]/bias_mag, gravity_vec[2]/bias_mag};
    
          float correction_vec[3];
          RotationVectorAtoB(a_norm, bias_vec_norm, correction_vec);
          for (int i = 0; i < 3; i++)
          correction_vec[i] *= kOrientationCorrectionFactor;
          float correction_quat[4];
          WtoQ(correction_vec, correction_quat);
    
          float orientation_quat_cpy[4] = {orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3]};
          QuaternionMultiplication(correction_quat, orientation_quat_cpy, orientation_quat);
        }
      }
  
      // Calculate global_accel.
      float global_accel_vec[3];
      if (false) {
        float sample_accel_vec[3] = {sample_rxed.ax, sample_rxed.ay, sample_rxed.az};
        MultiplyQuaternionByVector(orientation_quat, sample_accel_vec, global_accel_vec); 
        global_accel_vec[0] -= gravity_vec[0];
        global_accel_vec[1] -= gravity_vec[1];
        global_accel_vec[2] -= gravity_vec[2];
      }
  
      // Calculate highpass_accel_vec (unused currently).
      static float highpass_accel_vec[3] = {0};
      if (false) {
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
      if (false) {
        for (int i = 0; i < 3; i++) {
          if (is_biased)
            global_vel_vec[i] = 0;
          else
            global_vel_vec[i] += global_accel_vec[i]/1024.f;
        }
      }
  
      // Calculate highpass_vel_vec.
      static float highpass_vel_vec[3] = {0};
      if (false) {
        static float prev_sample[3] = {0};
        float k;
  
        if (is_moving)
          k = 0.999f;
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
  
      // Calculate yaw, roll, pitch, and their 1-sample derivatives.
      float R[3][3]; // Rotation vector.
      float yaw=0, roll=0, pitch=0;
      float ddx_yaw=0, ddx_roll=0, ddx_pitch=0;
      {
        RfromQ(R, orientation_quat);
        yaw   =  atan2f( R[1][0], R[0][0]);
        roll  =  atan2f(-R[2][0], sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
        pitch = -atan2f( R[2][1], R[2][2]);
  
        // Take the derivative every n samples
        static const int n = 1;
        static int count = 0;
        if (count == n) {
          count = 0;
          static float prev_yaw = 0, prev_roll = 0, prev_pitch = 0;
          ddx_yaw  = (yaw - prev_yaw) / n;
          prev_yaw = yaw;
          ddx_roll  = (roll - prev_roll) / n;
          prev_roll = roll;
          ddx_pitch = (pitch - prev_pitch) / n;
          prev_pitch = pitch;
        }
        count++;
      }
  
      // Debug feature, print values intermittently.
      {
        static int count = 0;
        if (count == 1000) {
          count = 0;
          static const float k = 180.0/M_PI;
  
          //printf("%f %f %f\n%f %f %f\n%f %f %f\n\n",
          //  R[0][0], R[0][1], R[0][2], 
          //  R[1][0], R[1][1], R[1][2], 
          //  R[2][0], R[2][1], R[2][2] 
          //);
          //printf("\n%f %f %f\n", yaw*k, roll*k, pitch*k);
          //printf("\n%10f %10f %10f\n", global_accel_vec[0], global_accel_vec[1], global_accel_vec[2]);
          //printf("\n%10f %10f %10f\n", bias_final_ax, bias_final_ay, bias_final_az);
          //printf("%f\n\t%f\n\t%f\n\t%f\n",
          //  orientation_quat[0],
          //  orientation_quat[1],
          //  orientation_quat[2],
          //  orientation_quat[3]
          //);
          //printf("%f\n\t%f\n\t%f\n\t%f\n",
          //  acosf(orientation_quat[0])*2*180/M_PI,
          //  orientation_quat[1]/sinf(acosf(orientation_quat[0])),
          //  orientation_quat[2]/sinf(acosf(orientation_quat[0])),
          //  orientation_quat[3]/sinf(acosf(orientation_quat[0]))
          //);
          //printf("%3.2f %3.2f %3.2f\n", orientation_vec[0], orientation_vec[1], orientation_vec[2]);
          //printf("%3.2f %3.2f %3.2f\n", highpass_accel_vec[0], highpass_accel_vec[1], highpass_accel_vec[2]);
          //printf("%3.2f %3.2f %3.2f\n", global_vel_vec[0], global_vel_vec[1], global_vel_vec[2]);
          //printf("%3.2f %3.2f %3.2f\n", highpass_vel_vec[0], highpass_vel_vec[1], highpass_vel_vec[2]);
        }
        else {
          count++;
        }
      }
  
      // Change mouse movement globals (communicates with hid_task()).
      if (true)
      {
        // Note, units are in pixels.
        //gDXMouse += ddx_pitch  * 1920.0f; // Positive = right.
        //gDYMouse -= ddx_roll * 1080.0f; // Positive = down.
      }
  
      // Debug feature, unpulse at end of interrupt.
      gpio_put(kDebugPin2, 1);
    }

    // Pixart sensor interrupt checking.
    if (gpio_get(kPixartIntPin) == false) {
      // Pulse a debug pin.
      gpio_put(kDebugPin1, 0);

      // Get all Optical data and send to spi_inbuf.
      spi_write_read_blocking(spi1, pixart_spi_outbuf, pixart_spi_inbuf, kPixartSpiBufLen);

      int8_t optical_x = pixart_spi_inbuf[3];
      int8_t optical_y = pixart_spi_inbuf[5];
      
      // Update mouse globals
      gDXMouse -= optical_y;
      gDYMouse -= optical_x;

      // Unpulse a debug pin.
      gpio_put(kDebugPin1, 1);
    }

    // UART/TTL processing.
    uint8_t byte;
    if (queue_try_remove(&gByteQueue, &byte)) {
      // Code below is executed a few times every "5ish" milliseconds.

      static int data_counter = 0; // Stores where we are in a transmission (ie. SYNC = 0, MBDATA = 1, IMUDATA = 2-7, etc.)
      static uint8_t temp = 0;

      // SYNC byte is signaled with "byte" MSB being 1.
      if (byte & 0x80)
        data_counter = 0;

      switch (data_counter) {
        case 0: {
          break;
        }
        // Mouse button data byte.
        case 1: {
          gMB1 = 0b01000000 & byte;
          gMB2 = 0b00100000 & byte;
          gMB3 = 0b00010000 & byte;
          int8_t scrl_ups = 0b00001111 & byte;

          // Restore 2's complement and add to global var.
          scrl_ups <<= 4;
          scrl_ups >>= 4;
          gScrlUps += scrl_ups;
          break;
        }
        
        // int14 head roll value.
        case 2: {
          temp = byte;
          break;
        }
        case 3: {
          gHeadRoll = (temp & 0x7f) << 7;
          gHeadRoll += byte & 0x7f;
          gHeadRoll <<= 2;
          gHeadRoll >>= 2;
          //printf("r=%04d\t", gHeadRoll);
          break;
        }
        
        // int14 head pitch value.
        case 4: {
          temp = byte;
          break;
        }
        case 5: {
          gHeadPitch = (temp & 0x7f) << 7;
          gHeadPitch += byte & 0x7f;
          gHeadPitch <<= 2;
          gHeadPitch >>= 2;
          //printf("p=%04d\t", gHeadPitch);
          break;
        }
        
        // int14 head yaw value.
        case 6: {
          temp = byte;
          break;
        }
        case 7: {
          gHeadYaw = (temp & 0x7f) << 7;
          gHeadYaw += byte & 0x7f;
          gHeadYaw <<= 2;
          gHeadYaw >>= 2;
          //printf("y=%04d\n", gHeadYaw);
          break;
        }

        default: {
          // This error can occur a few times at startup but shouldn't otherwise. 
          break;
        }
      }


      data_counter++;
    }
  }
}

// USB HID send.
static void send_hid_report(uint8_t report_id) {
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;

  switch (report_id) {
    case REPORT_ID_MOUSE: {
      // Calculate mode toggle.
      static int mouse_mode = 0;
      static bool prev_mb3 = 0;
      if (!prev_mb3 && gMB3) {
        mouse_mode++;
        if(mouse_mode == 2)
          mouse_mode = 0;
        LedTask(true);
      }
      prev_mb3 = gMB3;

      switch (mouse_mode) {
      case 0: {
          // Calculate amount to move cursor / "subtract" from gDXMouse & gDYMouse.
          // turn to ints and manually saturate.
          const int kFactor = 1; // To divide output for useability.
          int dx = gDXMouse / kFactor;
          int dy = gDYMouse / kFactor;
          if (dx > 127)
            dx = 127;
          if (dx < -128)
            dx = -128;
          if (dy > 127)
            dy = 127;
          if (dy < -128)
            dy = -128;
    
          // Create mouse button byte. [7to5=0][mb5][mb4][mb3][mb2][mb1].
          uint8_t mouse_buttons = (gMB1) + (gMB2<<1) + (gMB3<<2);
        
          // Move mouse.
          tud_hid_mouse_report(REPORT_ID_MOUSE, mouse_buttons, dx, dy, gScrlUps, 0);
        
          // Subtract the amount moved.
          gDXMouse -= dx * kFactor;
          gDYMouse -= dy * kFactor;
          gScrlUps = 0;
          break;
        }
        case 1: {
          static int16_t center_x = 0;
          static int16_t center_y = 0;

          //hid_abs_mouse_report report = 0; 
          //
          //tud_hid_abs_mouse_report(REPORT_ID_MOUSE, 0, center_x, center_y, 0, 0);
          break;
        }
      }
      break;
    }
    
    case REPORT_ID_GAMEPAD: {
      // Same process as above.
      const int kFactor = 5000;
      int dx = gDXGamepad / kFactor;
      int dy = gDYGamepad / kFactor;
      if (dx > 127)
        dx = 127;
      if (dx < -128)
        dx = -128;
      if (dy > 127)
        dy = 127;
      if (dy < -128)
        dy = -128;

      hid_gamepad_report_t report = {
        .x  = dx,
        .y  = dy,
        .z  = 0,
        .rz = 0,
        .rx = 0,
        .ry = 0,
        .hat = 0,
        .buttons = 0
      };

      //tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

      gDXGamepad -= dx * kFactor;
      gDYGamepad -= dy * kFactor;
  
      break;
    }
  }
}

// Invoked when device is mounted.
void tud_mount_cb(void) {

}

// Invoked when device is unmounted.
void tud_umount_cb(void) {

}

// Invoked when usb bus is suspended.
// remote_wakeup_en : if host allows us to perform remote wakeup.
// Within 7ms, device must draw an average of current less than 2.5 mA from bus.
void tud_suspend_cb(bool remote_wakeup_en) {

}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {

}

// Calls send_hid_report if it has been long nough since the last call.
void hid_task(void) {
  // Poll every 10ms.
  const uint32_t interval_ms = 10;
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
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {
  (void) instance;
  (void) len;

  if (report[0] == REPORT_ID_MOUSE)
    send_hid_report(REPORT_ID_GAMEPAD);
}

// Invoked when received GET_REPORT control request. Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
  return 0;
}

// Invoked when received SET_REPORT control request or received data on OUT endpoint ( Report ID = 0, Type = 0 ).
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {

}

// Call with true to initiaze a blink, call with false to turn off the LED if the blink is done. 
void LedTask(bool start_blink) {
  const int millis_on = 10;
  static int start_millis = 0;

  if (start_blink) {
    board_led_write(true);
    start_millis = board_millis();
  }

  if (board_millis() - start_millis >= millis_on) {
    board_led_write(false);
  }
}

// Initialize SPIbus for IMU.
void ImuInit() {
  // SPI interupt pin initialization.
  gpio_init(kImuIntPin);
  gpio_set_dir(kImuIntPin, GPIO_IN);
  gpio_disable_pulls(kImuIntPin);
  gpio_pull_up(kImuIntPin);
  
  // SPI bus initialization. (1MHZ)(0b1 = high/low?)(phase = ??)(most sig. bit first)
  spi_init(spi0, 1000 * 1000);
  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(kImuRxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kImuSckPin, GPIO_FUNC_SPI); 
  gpio_set_function(kImuTxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kImuCsnPin, GPIO_FUNC_SPI); 

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

  out_buf[0] = kAccelConfig0;
  out_buf[1] = 0b01000110; // Change FS from +-16g to +-4g.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
  out_buf[0] = kGyroConfig0;
  out_buf[1] = 0b01000110; // Change FS from +-2000dps to +-500dps.
  spi_write_read_blocking(spi0, out_buf, in_buf, 2);
}

// Initialize SPIbus for optical sensor. 
void PixartInit() {
  // Init motion pin.
  gpio_init(kPixartIntPin);
  gpio_set_dir(kPixartIntPin, GPIO_IN);
  gpio_disable_pulls(kPixartIntPin);
  gpio_pull_up(kPixartIntPin);

  // Init Pixart reset pin.
  gpio_init(kPixartResetnPin);
  gpio_set_dir(kPixartResetnPin, GPIO_OUT);
  gpio_disable_pulls(kPixartResetnPin);
  gpio_put(kPixartResetnPin, 1);
  
  // SPI bus initialization. (1MHZ)(0b1 = high/low?)(phase = ??)(most sig. bit first)
  spi_init(spi1, 1000 * 1000);
  spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(kPixartRxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kPixartSckPin, GPIO_FUNC_SPI); 
  gpio_set_function(kPixartTxPin, GPIO_FUNC_SPI); 
  gpio_set_function(kPixartCsnPin, GPIO_FUNC_SPI); 

  // Create the instruction buffer, imu_spi_outbuf, and the register rx buffer, spi_inbuf.
  const int kBufLen = 10;
  uint8_t spi_outbuf[kBufLen], spi_inbuf[kBufLen];

  {
    // Do some one-time SPI writes. (CONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIALCONFIDENTIAL)
    spi_outbuf[0] = 0x80 | 0x41;
    spi_outbuf[1] = 0xba;
    //spi_write_read_blocking(spi1, spi_outbuf, spi_inbuf, 2);
    sleep_ms(1);
    spi_outbuf[0] = 0x80 | 0x3a;
    spi_outbuf[1] = 0x5a;
    spi_write_read_blocking(spi1, spi_outbuf, spi_inbuf, 2);
  }
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
void CrossProduct(float a[3], const float b[3], const float c[3]) {
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

float VectorLength(const float a[3]) {
  return sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

float VectorDot(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Compute the rotation vector that rotates unit length vector a to unit length
// vector b. This handles the singularity when a == b.
void RotationVectorAtoB(const float a[3], const float b[3], float result[3]) {
  float acb[3];
  CrossProduct(acb, a, b);
  float acb_length = VectorLength(acb);
  float scale = 1.0f / Sinc(atan2(acb_length, VectorDot(a, b)));
  result[0] = acb[0] * scale;
  result[1] = acb[1] * scale;
  result[2] = acb[2] * scale;
}

void RfromQ (float R[3][3], const float q[4]) {
  // q = (s,vx,vy,vz)
  float qq1 = 2*q[1]*q[1];
  float qq2 = 2*q[2]*q[2];
  float qq3 = 2*q[3]*q[3];
  R[0][0] = 1 - qq2 - qq3;
  R[0][1] = 2*(q[1]*q[2] - q[0]*q[3]);
  R[0][2] = 2*(q[1]*q[3] + q[0]*q[2]);

  R[1][0] = 2*(q[1]*q[2] + q[0]*q[3]);
  R[1][1] = 1 - qq1 - qq3;
  R[1][2] = 2*(q[2]*q[3] - q[0]*q[1]);

  R[2][0] = 2*(q[1]*q[3] - q[0]*q[2]);
  R[2][1] = 2*(q[2]*q[3] + q[0]*q[1]);
  R[2][2] = 1 - qq1 - qq2;
}

void HeadBoardReqTask() {
  // Poll every 5ms.
  const uint32_t interval_ms = 5;
  static uint32_t start_ms = 0;
  if ( board_millis() - start_ms < interval_ms)
    return; // not enough time
  start_ms += interval_ms;

  // Following code is ran every interval_ms.
  uint8_t uart_out[1] = {0b10000000};
  uart_write_blocking(uart1, uart_out, 1);
}

void Uart1InterruptHandler() {
  // Loop until RX FIFO empty.
  while (uart_is_readable(uart1)) {
    // Capture byte that was RXed.
    uint16_t dr = uart_get_hw(uart1)->dr; // Reading from this register pops it from the FIFO.
    uint8_t rsr = dr >> 8; // RSR contains a list of uart errors described in the pico sdk github source.
    uint8_t byte = (uint8_t)dr;
    if (queue_try_add(&gByteQueue, &byte) == false) {
      // The queue is full.
      // Do nothing. It will fix itself.
    }
  }
}
