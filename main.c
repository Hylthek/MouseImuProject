#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

enum  {
  BLINK_NOT_MOUNTED = 250, // Device not mounted
  BLINK_MOUNTED = 1000,    // Device mounted
  BLINK_SUSPENDED = 2500,  // Device is suspended

  DEBUG_PIN1 = 12,
  DEBUG_PIN2 = 13,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);
bool VirtualButtonOutput();

int main(void) {
  // Init debug printfs.
  stdio_uart_init_full(uart0, 115200, DEBUG_PIN1, DEBUG_PIN2);

  // Init tinyusb and LED function.
  tusb_init();
  board_init();

  // Blink LED
  bool led_on = 0;
  for (int i = 0; i < 10; i++) {
    board_led_write(led_on);
    led_on = !led_on;
    sleep_ms(100);
  }

  // Main loop.
  while (true) {
    printf("\ntask ");

    tud_task(); // tinyusb device task
    //led_blinking_task(); // Toggles LED at speed depending on USB state.
    hid_task();
  }
}

// Invoked when device is mounted.
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted.
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended.
// remote_wakeup_en : if host allows us to perform remote wakeup.
// Within 7ms, device must draw an average of current less than 2.5 mA from bus.
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en; // <-means variable is not used.
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// USB HID
static void send_hid_report(uint8_t report_id, uint32_t btn) {
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;

  static double theta = 0;
  int8_t delta_x = 15 * cos(theta);
  int8_t delta_y = 15 * sin(theta);

  // Mouse moves in circles
  if (btn) {
    tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta_x, delta_y, 0, 0);
    theta += 0.5;
  }
  else
    tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, 0, 0, 0, 0);
}

// Every 10ms, we will sent 1 "mouse HID profile" report.
void hid_task(void) {
  // Poll every 10ms.
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  bool const btn = VirtualButtonOutput(); // Simulates a button press because there are no buttons on version 1 of my board.

  // Remote wakeup.
  if ( tud_suspended() && btn ) {
    // Wake up host if we are in suspend mode.
    // and REMOTE_WAKEUP feature is enabled by host.
    tud_remote_wakeup();
  }
  else {
    // Send the report.
    send_hid_report(REPORT_ID_MOUSE, btn);
  }
}

// Invoked when sent REPORT successfully to host
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {
  (void) instance;
  (void) report;
  (void) len;
}

// Invoked when received GET_REPORT control request.
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 ).
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}

// Blinking task.
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

// Presses the virtual button for half a sec every sec.
bool VirtualButtonOutput() {
  if (board_millis() % 1000 >= 0) {
    board_led_write(true);
    return true;
  }
  else {
    board_led_write(false);
    return false;
  }
}
