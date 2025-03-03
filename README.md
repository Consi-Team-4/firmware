# rp2040-freertos-template
This is a template project for developing FreeRTOS based applications on Raspberry Pi RP2040 based boards. This template uses the "official" RP2040 port from the Raspberry Pi Foundation. My previous repo used the generic Cortex M0 port so this one corrects that. Previous Repo retained as tutorials rely on it. 

Unmodified, this project will spawn a single task to rapidly blink the LED on and off.


# Note from Gideon
The initial settings weren't working correctly for some reason. I think related to freeRTOS settings changing.
Fixed the freeRTOSConfig.h

I added pico-i2c-dma so i2c transactions don't block the rest of our code

Dynamic memory allocation is enabled (pico-i2c-dma uses it and I didn't feel like rewriting that), but static allocation is generally preferred where reasonable.