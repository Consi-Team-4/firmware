# Car Firmware
This is the main firmware repo for the active suspension car.


# Libraries
- FreeRTOS
- pico-i2c-dma

### FreeRTOS note:
Dynamic memory allocation is enabled (pico-i2c-dma uses it and I didn't feel like rewriting that), but static allocation is generally preferred where reasonable.


Task priorities:

IMU: 4
Lidar: 4
Timer: 3 (controller)
Bluetooth: 2
Console: 2
Heartbeat: 1
Idle: 0




# Uploading code to microcontroller
Plug into the USB port.
While holding down the button large button near the nano, tap the small white button on top of the board.
Drag build/src/firmware.uf2 onto that drive. It'll close once it's done copying.
And Code should be uploaded now!

If you're ever uncertain, upload arduino's blink.uf2 to reset things.
