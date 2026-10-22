# setMotorStop(id)

Set the motor to stop

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18 |

### Example

Use the uKit Explore v2 board to control DC Motor ID-1 to rotate at a constant speed of 80 counterclockwise, stop DC Motor ID-1 after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setMotorTurnAdj(1,-80,0xffff);    
    delay(400);    
    setMotorStop(1);
}

void loop() { 
   }
```
