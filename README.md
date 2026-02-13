--------------------------------------------------------------------------------------------------------------------------------
    ==== M5-Stack Core Basic - Bluetooth Hat-TRACKER-MOUSE (HAT-MOUSE) - with the ICM-20948 IMU Sensor with DMP support ====
  --------------------------------------------------------------------------------------------------------------------------------
  Hardware: ESP32: M5Stack Core Basic & Sensor: ICM-20948 (on sparkfun breakout board) - Quat9(Gyro+Acc+Mag) => X and Y will be sent out
  Project descripton: A simple wireless Hat-Tracker-Device (MOUSE version) for BluetoothLE to be attatched on your head to control
  yaw and nick movements with an emulated BT-HID-Mouse for your PC. (NOTE !) There is a better gamepadverion too that is more useful
  because its faster, sends absolute values (incl. Z) where a mouse only sends X an Y deleta-move values.
  So this version is only for the cases where you prefer a mouse over a joystick-input for whatever reason you have.
  Initially it was designed for a RC-Simulators for planes, helis, drones to get rid of the non resalitic auto-follow cama.

  Sends 2 axis (X,Y) Has a display: 320*240px, 10s screen-off-delay when BT-connected, 2x yellow/blue StatusLed with a NeoPixelBar
  Menu-Buttons for: [B]=lock-in LeftMouseButton, [C]=toggle Tilt-Lock, long-press[C]=re-center sensor
  You can turn-off the device by long pressing [A] or with the integrated M5-PowerButton or whait 6min with BT-connection off.
  The sensor technically takes up 10-30sek to warm and stop drifting, escpecially when its not facing a default position like vertical.
  											    @by frittna - 2.Feb 2026 Arduino IDE 1.8.19
