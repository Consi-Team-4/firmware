#ifndef DRIVE_ESC_H
#define DRIVE_ESC_H

void escSetup();
void escTaskFunc(void *params);
void serialCommandTaskFunc(void *params);

#endif