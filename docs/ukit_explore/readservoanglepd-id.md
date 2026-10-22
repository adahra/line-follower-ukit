# readServoAnglePD(id)

Read the servo angle while the servo is powered off (can be moved)

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18 |

### Return Values

| Parameters | Type | Description                           |
| ---------- | ---- | ------------------------------------- |
| number     | int  | Servo’s angle, Range from -118 \~ 118 |

### 例子

se uKit Explore v2 board to control servo ID-1 to rotate to -20 ° in 400ms. Then read and print the servo’s angle every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setServoAngle(1,-20,400);    
    delay(400);
}

void loop() {    
    Serial.println(readServoAnglePD(1));    
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
    Serial.println(readServoAnglePD(1));    
    setServoAngle(1,Agnle,100);    
    delay(100);    
    if(Angle>=118){	
        Angle=118;    
    }
}
```

Results: the new angle of the Servo Motor can be read after the angle of the Servo Motor is read and the steering gear is broken.
