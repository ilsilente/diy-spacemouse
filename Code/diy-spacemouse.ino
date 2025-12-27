#include <Wire.h>
#include "TLx493D_inc.hpp"
#include <SimpleKalmanFilter.h>
#include <OneButton.h>
#include <Adafruit_TinyUSB.h>

// USB HID
Adafruit_USBD_HID usb_hid;
enum { RID_KEYBOARD=1, RID_MOUSE };

// HID descriptor: keyboard + mouse
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE   (HID_REPORT_ID(RID_MOUSE))
};

// Magnetometer
using namespace ifx::tlx493d;
TLx493D_A1B6 mag(Wire1, TLx493D_IIC_ADDR_A0_e);

// Kalman filters
SimpleKalmanFilter xFilter(1,1,0.2);
SimpleKalmanFilter yFilter(1,1,0.2);
SimpleKalmanFilter zFilter(1,1,0.2);

// Offsets
float xOffset=0, yOffset=0, zOffset=0;
float xCurrent=0, yCurrent=0, zCurrent=0;

// Calibration
int calSamples=50;
float sensitivity=5; 
float xyThreshold=0.4;
float zThreshold = xyThreshold*2.5;

// Buttons
OneButton button1(27,true); // Left click
OneButton button2(24,true); // Toggle orbit

// Keyboard placeholder
uint8_t key_none[6] = {HID_KEY_NONE};

// State
bool middle=false;          // Middle mouse held
bool orbitModeActive=false; // Toggle orbit mode

// Calibration
void calibrate() {
  double x,y,z;
  Wire1.begin();
  mag.setPowerPin(15,OUTPUT,INPUT,HIGH,LOW,0,250000);
  mag.begin();

  for(int i=0;i<calSamples;i++){
    mag.getMagneticField(&x,&y,&z);
    xOffset+=x; yOffset+=y; zOffset+=z;
    Serial.println("calibrate() -> mag.getMagneticField()");
    Serial.println(x, DEC);
    Serial.println(y, DEC);
    Serial.println(z, DEC);
    Serial.println();
    delay(5);
  }
  xOffset/=calSamples;
  yOffset/=calSamples;
  zOffset/=calSamples;

    Serial.println("calibrate() -> Offsets:");
    Serial.println(xOffset, DEC);
    Serial.println(yOffset, DEC);
    Serial.println(zOffset, DEC);
    Serial.println();

}

// Map magnetometer to HID range
int mapAxis(float value){
  int range = 127;
  return constrain((int)(value*sensitivity), -range, range);
}

// Button1: Left mouse click
void btn1() {
  usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_LEFT);
  delay(10);
  usb_hid.mouseButtonRelease(RID_MOUSE);
}

// Button1 long press: CTRL + SHIFT + H
void btn1Home() {
  uint8_t keycode[6] = { HID_KEY_H, 0, 0, 0, 0, 0 };

  usb_hid.keyboardReport(
    RID_KEYBOARD,
    KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT,
    keycode
  );

  delay(20);

  // Rilascio tasti
  usb_hid.keyboardRelease(RID_KEYBOARD);
}

// Button2: Toggle 360 orbit
void btn2() {
  orbitModeActive = !orbitModeActive;

  if (orbitModeActive) {
    // Start orbit: hold Shift + Middle mouse
    usb_hid.keyboardReport(RID_KEYBOARD, KEYBOARD_MODIFIER_LEFTSHIFT, key_none);
    usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
    middle = true;
  } else {
    // Stop orbit: release Shift + Middle
    usb_hid.mouseButtonRelease(RID_MOUSE);
    usb_hid.keyboardRelease(RID_KEYBOARD);
    middle = false;
  }
}

// Read magnetometer and send HID
void getMagnet(){
  double x,y,z;
  mag.getMagneticField(&x,&y,&z);

  xCurrent=xFilter.updateEstimate(x-xOffset);
  yCurrent=yFilter.updateEstimate(y-yOffset);
  zCurrent=zFilter.updateEstimate(z-zOffset);

  int xMove = mapAxis(xCurrent);
  int yMove = mapAxis(yCurrent);

    Serial.println("getMagnet() -> moves:");
    Serial.print("xMove: ");  
    Serial.println(xMove, DEC);  
    Serial.print("yMove: ");  
    Serial.println(yMove, DEC);  

  if(orbitModeActive){
    // 360 orbit mode: Shift + Middle held
    usb_hid.mouseReport(RID_MOUSE,MOUSE_BUTTON_MIDDLE,xMove,-yMove,0,0);
  } else {
    // Regular pan mode: move mouse proportionally
    usb_hid.mouseReport(RID_MOUSE,0,xMove,-yMove,0,0);
  }
}

// Setup
void setup(){
    usb_hid.setReportDescriptor(desc_hid_report,sizeof(desc_hid_report));
    usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
    usb_hid.setPollInterval(2);
    usb_hid.begin();

    if(TinyUSBDevice.mounted()){
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    Serial.begin(115200);
    while(!Serial);

    calibrate();

    button1.attachClick(btn1);              // short click -> mouse left

    // The default long press for OneButton is 800 ms
    button1.attachLongPressStart(btn1Home); // long click -> HOME

    button2.attachClick(btn2);              // orbit mode ON/OFF
}

// Loop
void loop(){
    button1.tick(); button2.tick();

#ifdef TINYUSB_NEED_POLLING_TASK
    TinyUSBDevice.task();
#endif

    if(!TinyUSBDevice.mounted()) return;

    getMagnet();
    delay(5);
}
