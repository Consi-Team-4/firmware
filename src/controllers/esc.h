#ifndef ESC_H
#define ESC_H

void escSetup();
void setEscTaskFunc(uint pwm_us);
void serialCommandTaskFunc(void *params);

#endif