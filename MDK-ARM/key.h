#ifndef __KEY_H
#define __KEY_H

#include "main.h"

typedef enum {
    KEY_NONE = 0,
    KEY_PRESS_1,
    KEY_PRESS_2,
    KEY_PRESS_3,
    KEY_PRESS_4
} KeyEvent;

void     Key_Init(void);
KeyEvent Key_Scan(void);

#endif
