# setServoTurn(id,dir, speed)

Control the servo motor to rotate 360 °

### Parameter

| Parameters | Type          | Description                                                                               |
| ---------- | ------------- | ----------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18                                                 |
| dir        | int           | Control the direction of the servo. 0 represents clockwise 1 represents counter clockwise |
| speed      | int           | Control the rotation speed of the servo, range 0 \~ 255                                   |

### Example

Use uKit Explore v2 board to control the Servo ID-1 to rotate counterclockwise at speed of 80

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setServoTurn(1,1,80); 
}
void loop() {
    }
```

Use the uKit Explore v2 board to control the Servo ID-1 to rotate counterclockwise with the speed slowly increases from 0 to the maximum speed.

```c
#include "uKitExplore2.h"
int speed=0;
void setup() {    
  Initialization();
}

void loop() {    
  speed++;    
  if(speed>=255){	
    seed=255;  
  }    
  setServoTurn(1,1,speed);
}
```
