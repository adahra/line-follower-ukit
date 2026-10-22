> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/motor/readmotorspeed-id.md).

# readMotorSpeed(id)

Read DC Motor speed

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18 |

### Return Values

| Parameters | Type | Description                                                                                                                                      |
| ---------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| number     | int  | Returns the motor speed, range -140 \~ 140 r / min. The value read is negative for counterclockwise rotation and positive for clockwise rotation |

### Example

Use uKit Explore v2 board to control the DC Motor ID-1 to rotate clockwise with constant speed of 80 rpm all the time. Then read and print the motor speed every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setMotorTurnAdj(1,80,0xffff);
}

void loop() {    
    Serial.println(readMotorSpeed(1));    
    delay(400);
}
```

Use the uKit Explore v2 board to control DC Motor ID-1 to start rotate counterclockwise at a constant speed of -80. Add the speed every 200ms until a clockwise speed of 80 and print the speed every time.

```c
#include "uKitExplore2.h"
int Angle=-80;
void setup() {    
    Initialization();
}

void loop() {    
    Angle++;    
    Serial.println(readMotorSpeed(1));    
    setMotorTurnAdj(1,Angle,0xffff);	     
    delay(200);    
    if(Angle>=80){	
        Angle=80;    
    }
}
```
