# readServoAngleNPD(id)

Read the angle of the servo motor while the servo motor maintain its position (do not move it)

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18 |

### Return Values

| Parameters | Type | Description                           |
| ---------- | ---- | ------------------------------------- |
| number     | int  | Servo’s angle, Range from -118 \~ 118 |

### Example

se uKit Explore v2 board to control servo ID-1 to rotate to -20 ° in 400ms. Then read and print the servo’s angle every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setServoAngle(1,-20,400);    
    delay(400);
}

void loop() {    
    Serial.println(readServoAngleNPD(1));    
    delay(400);
}
```

Use uKit Explore v2 board to control the servo ID-1 from -118° and add 1° every 100ms at a speed of 100ms. Then read and print the servo’s angle.

```c
#include "uKitExplore2.h"
int Angle=-118;

void setup() {   
    Initialization();
}

void loop() {    
    Angle++;    
    Serial.println(readServoAngleNPD(1));    
    setServoAngle(1,Agnle,100);    
    delay(100);    
    if(Angle>=118){	
        Angle=118;    
    }
}
```
