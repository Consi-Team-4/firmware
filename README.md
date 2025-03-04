# rp2040-freertos-template
This is a template project for developing FreeRTOS based applications on Raspberry Pi RP2040 based boards. This template uses the "official" RP2040 port from the Raspberry Pi Foundation. My previous repo used the generic Cortex M0 port so this one corrects that. Previous Repo retained as tutorials rely on it. 

Unmodified, this project will spawn a single task to rapidly blink the LED on and off.


# Note from Gideon
The initial settings weren't working correctly for some reason. I think related to freeRTOS settings changing.
Fixed the freeRTOSConfig.h

I added pico-i2c-dma so i2c transactions don't block the rest of our code

Dynamic memory allocation is enabled (pico-i2c-dma uses it and I didn't feel like rewriting that), but static allocation is generally preferred where reasonable.


# Uploading code to microcontroller
Plug into the USB port.
If looking at the microcontroller so the USB is at the top, find the pin with the white soldermask in the bottom left corner. That pin is ground.
Short that pin to the pin one above and press the reset button. You should see RPI-RP2 show up as a drive in file explorer.
Un-short the pin.
Drag build/src/firmware.uf2 onto that drive. It'll close once it's done copying.
And Code should be uploaded now!

If you're ever uncertain, I have a basic blink.uf2 file I dowloaded from arduino to reset things.
