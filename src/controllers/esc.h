#ifndef ESC_H
#define ESC_H

void escSetup();
void escTaskFunc(void *params);
void serialCommandTaskFunc(void *params);

#endif