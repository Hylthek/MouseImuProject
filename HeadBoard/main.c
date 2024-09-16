#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/util/queue.h"


// Pins
enum {
  // RP2040 GPIO pins (no UART control).
  kImuRxPin =  16, // MISO
  kImuCsnPin = 17, // Chip select.
  kImuSckPin = 18, // SPI clock
  kImuTxPin =  19, // MOSI
  kImuIntPin = 20, // Goes low at IMU data ready.
  kImuClkinPin = 21, // Insert a 32.77kHz square wave to replace embedded IMU clock.
  kMB1Pin = 25,  // Put a pullup gpio input here.
  kMB2Pin = 23,  // Put a pullup gpio input here.
  kMB3APin = 10, // two pins for safety because there is a diode in-line.
  kMB3BPin = 11, // two pins for safety because there is a diode in-line.
  kScrlAPin = 22, // Three pins of a quadrature encoder.
  kScrlBPin = 2,  // Three pins of a quadrature encoder.
  kScrlCPin = 3,  // Three pins of a quadrature encoder.
  kTtlTxPin = 4, // For communicating with the other board (SE-UART).
  kTtlRxPin = 5, // For communicating with the other board (SE-UART).

  // Debugging pins.
  kDebugPin1 = 0,
  kDebugPin2 = 1,
  kLedPin = 24,
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

// Function declarations.
void ImuInit();
void LedTask(bool start_blink);
void WtoQ(float w[3], float q[4]);
void QuaternionMultiplication(float q_l[4], float q_r[4], float q_result[4]);
void MultiplyQuaternionByVector(float q[4], float v[3], float result[3]);
void RotationVectorAtoB(const float a[3], const float b[3], float result[3]);
float VectorLength(const float a[3]);
void RfromQ (float R[3][3], const float q[4]);
void Uart1InterruptHandler();

// Globals
queue_t gByteQueue; // A queue that stores the bytes rxed by the TTL UART port.

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

  // Init buttons and encoders
  {
    gpio_init  (kMB1Pin);
    gpio_init  (kMB2Pin);
    gpio_init (kMB3BPin);
    gpio_init(kScrlAPin);
    gpio_init(kScrlBPin);
    gpio_set_dir  (kMB1Pin, GPIO_IN);
    gpio_set_dir  (kMB2Pin, GPIO_IN);
    gpio_set_dir (kMB3BPin, GPIO_IN);
    gpio_set_dir(kScrlAPin, GPIO_IN);
    gpio_set_dir(kScrlBPin, GPIO_IN);
    gpio_disable_pulls  (kMB1Pin);
    gpio_disable_pulls  (kMB2Pin);
    gpio_disable_pulls (kMB3BPin);
    gpio_disable_pulls(kScrlAPin);
    gpio_disable_pulls(kScrlBPin);
    gpio_pull_up  (kMB1Pin);
    gpio_pull_up  (kMB2Pin);
    gpio_pull_up (kMB3BPin);
    gpio_pull_up(kScrlAPin);
    gpio_pull_up(kScrlBPin);

    // MB3A and ScrlC are set to GND
    gpio_init(kMB3APin);
    gpio_set_dir(kMB3APin, GPIO_OUT);
    gpio_disable_pulls(kMB3APin);
    gpio_put(kMB3APin, 0);
    gpio_init(kScrlCPin);
    gpio_set_dir(kScrlCPin, GPIO_OUT);
    gpio_disable_pulls(kScrlCPin);
    gpio_put(kScrlCPin, 0);
  }

  // IMU SPIbus initialization.
  // Pixart sensor SPIbus initialization.
  ImuInit();

  // Create the IMU instruction buffer, out_buf, and the register buffer, in_buf
  // Prepare a read instruction that outputs concurrent registers (IMU data).
  const int kImuSpiBufLen = 13;
  uint8_t imu_spi_outbuf[kImuSpiBufLen], imu_spi_inbuf[kImuSpiBufLen];
  imu_spi_outbuf[0] = (1 << 7) | kAccelDataX1;

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

  // Main loop.
  while (true) {
    // Routine LED check (arg is false = can only turn off led).
    LedTask(false);

    // Update button/switch states.
    // Note, the following variables are only sent occasionally.
    // They must be static to account for info inbetween data sends.
    static bool mb1 = false, mb2 = false, mb3 = false;
    static int scrl_ups = 0; // Number of scroll ups (can be negative for scroll downs).
    {
      // Set mouse button vars.
      if (gpio_get(kMB1Pin) == 0)
        mb1 = true;
      if (gpio_get(kMB2Pin) == 0)
        mb2 = true;
      if (gpio_get(kMB3BPin) == 0)
        mb3 = true;

      // Derive quadrature states from pins. States 0 & 2 represent stable wheel positions while 1 & 3 represent intermediary ones.
      int state = 0;
      static int inter_state = 0;
      static int origin_state = 0;
      bool a = gpio_get(kScrlAPin);
      bool b = gpio_get(kScrlBPin);
      if (a == 0 && b == 0)
        state = 0;
      if (a == 0 && b == 1)
        state = 1;
      if (a == 1 && b == 1)
        state = 2;
      if (a == 1 && b == 0)
        state = 3;
  
      // Set scrl vars.
      if (origin_state == 0 && inter_state == 1 && state == 2 ||
          origin_state == 2 && inter_state == 3 && state == 0) {
        scrl_ups--;
      }
      if (origin_state == 0 && inter_state == 3 && state == 2 ||
          origin_state == 2 && inter_state == 1 && state == 0) {
        scrl_ups++;
      }

      // Set origin and inter states if applicable.
      if (state == 0 || state == 2)
        origin_state = state;
      else
        inter_state = state;
    }
    
    // IMU interrupt checking.
    // Note, the following variables are only sent occasionally.
    // They must be static to account for info inbetween data sends.
    float roll_out, pitch_out, yaw_out;
    if (gpio_get(kImuIntPin) == false) {
      // Debug feature, pulse debug pin on interrupt negedge.
      //gpio_put(kDebugPin2, 0);
  
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
      bool is_moving;
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
      bool is_hovering;
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
      float orientation_vec[3];
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
        // Reorient orientation quat for roll, pitch, yaw calcs alligned to board.
        float reorientation_quat[4] = {1, 0, 0, 0}; //90deg rotation along Z axis. 
        float new_orientation_quat[4];
        QuaternionMultiplication(orientation_quat, reorientation_quat, new_orientation_quat);
        
        RfromQ(R, new_orientation_quat);
        yaw   =  atan2f( R[1][0], R[0][0]);
        pitch =  atan2f(-R[2][0], sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
        roll  = -atan2f( R[2][1], R[2][2]);

        // Correct the axis that we reoriented.
        yaw -= 0;
  
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

      // Update output variables.
      // Note that output is not sent every IMU interrupt (most info is discarded if not explicitly stored).
      roll_out = roll;
      pitch_out = pitch;
      yaw_out = yaw;

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
  
      // Debug feature, unpulse at end of interrupt.
      //gpio_put(kDebugPin2, 1);
    }

    // UART processing.
    // I could've used a while loop to process all of the bytequeue,
    // but popping just one is probably fast enough, and potentially processing
    // a lot of data also seems like bad practice.
    uint8_t byte;
    if (queue_try_remove(&gByteQueue, &byte)) {
      // Code below is executed a few times every "5ish" milliseconds.
      if (byte == 0x80) {
        // Make mouse button data frame.
        uint8_t buttons = (mb1<<6) + (mb2<<5) + (mb3<<4) + (0x0f & scrl_ups); // [0][mb1][mb2][mb3][(int4_t)scrl_ups]

        // Make IMU dataframe.
        const float kMultiplier = 180/M_PI * 4 * 10; // Convert to degrees, bit-shift left by 2, add one decimal point of precision.
        int16_t roll = roll_out * kMultiplier;
        uint8_t roll_14[2]; // Store the upper 14 bits of the int16 into two bytes.
        roll_14[0] = (roll >> 9) & 0x7f;  // Store the upper 7 bits.
        roll_14[1] = (roll >> 2) & 0x7f;  // Store the lower 7 bits.

        int16_t pitch = pitch_out * kMultiplier;
        uint8_t pitch_14[2]; // Store the upper 14 bits of the int16 into two bytes.
        pitch_14[0] = (pitch >> 9) & 0x7f;  // Store the upper 7 bits.
        pitch_14[1] = (pitch >> 2) & 0x7f;  // Store the lower 7 bits.

        int16_t yaw = yaw_out * kMultiplier;
        uint8_t yaw_14[2]; // Store the upper 14 bits of the int16 into two bytes.
        yaw_14[0] = (yaw >> 9) & 0x7f;  // Store the upper 7 bits.
        yaw_14[1] = (yaw >> 2) & 0x7f;  // Store the lower 7 bits.

        // Send data back to BodyBoard (no data byte can have MSB = 1).
        uart_write_blocking(uart1, &buttons, 1);
        uart_write_blocking(uart1, roll_14, 2);
        uart_write_blocking(uart1, pitch_14, 2);
        uart_write_blocking(uart1, yaw_14, 2);

        // Reset static variables.
        mb1 = false;
        mb2 = false;
        mb3 = false;
        scrl_ups = 0;
      }
    }
  }
}

// Call with true to initiaze a blink, call with false to turn off the LED if the blink is done. 
void LedTask(bool start_blink) {
  const int millis_on = 10;
  static int start_millis = 0;

  if (start_blink) {
    gpio_put(kLedPin, 1);
    start_millis = get_absolute_time()/1000;
  }

  if (get_absolute_time()/1000 - start_millis >= millis_on) {
    gpio_put(kLedPin, 0);
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

void Uart1InterruptHandler() {
  // Loop until RX FIFO empty.
  while (uart_is_readable(uart1)) {
    // Capture byte that was RXed.
    uint16_t dr = uart_get_hw(uart1)->dr; // Reading from this register pops it from the FIFO.
    uint8_t rsr = dr >> 8; // RSR contains a list of uart errors described in the pico sdk github source.
    uint8_t byte = (uint8_t)dr;
    if (queue_try_add(&gByteQueue, &byte) == false) {
      // Do nothing.
      // It will fix itself.
    }
  }
}
