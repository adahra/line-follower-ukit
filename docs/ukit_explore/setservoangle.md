> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/servo/setservoangle.md).

# setServoAngle(id,angle,times)

Control the servo motor to rotate to a specified angle (non-blocking)

### Parameter

| Parameters | Type          | Description                                                |
| ---------- | ------------- | ---------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18                  |
| angle      | int           | Control the rotation angle of the servo, range -118 \~ 118 |
| time       | int           | Servo running time, unit ms, range 20 \~ 5000              |

### Example

Use uKit Explore v2 board to control the servo motor ID-1 to rotate to 30 ° within 400ms, then to -30 ° within 400ms

```c
#include "uKitExplore.h"
void setup() {    
    Initialization();
    
}

void loop() {    
    setServoAngle(1,30,400);    
    delay(400);    
    setServoAngle(1,-30,400);    
    delay(400);
}
```

Use uKit Explore v2 board to simultaneously control the servos ID-1 and ID-2 to rotate to 30 ° within 400ms, and then to -30 ° within 400ms at the same time.

```c
#include "uKitExplore.h"
void setup() {    
    Initialization();
} 

void loop() {    
    setServoAngle(1,30,400);    
    setServoAngle(2,30,400);    
    delay(400);    
    setServoAngle(1,-30,400);    
    setServoAngle(2,-30,400);    
    delay(400);
}
```
