> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/motor/setmotorturnadj.md).

# setMotorTurnAdj(id, speed,time)

Set the motor to rotate at constant speed (non-blocking)

### Parameter

| Parameters | Type          | Description                                                                                                                                                      |
| ---------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18                                                                                                                        |
| speed      | int           | Set motor rotation speed. Positive numbers indicate clockwise rotation, negative numbers indicate counterclockwise rotation, unit: r/ min, range: -140 \~ 140    |
| time       | int           | Set the rotation time: unit: 1ms, when the motor rotates for the specified time, the motor automatically stops rotating, 0xFFFF means that it will always rotate |

###

{% hint style="info" %}
About the difference between speed and PWM mode: speed mode is a closed-loop control that allows the motor to maintain a certain speed. The disadvantage: forward and backward switching requires 100ms waiting. The PWM mode is simply to control the rotation speed with voltage, and the forward and backward rotation can be switched without waiting. Disadvantages: The speed will be changed by the weight of the car.
{% endhint %}

### Example

Use uKit Explore v2 board to control the DC Motor ID-1 to rotate counterclockwise all the time at the speed of 80.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setMotorTurnAdj(1,-80,0xffff);
     
}

void loop() {   
 }
```

Use the uKit Explore v2 board to control the DC Motor ID-1 to rotate counterclockwise all the time with the speed slowly increases from 0 to the maximum speed.

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
    setMotorTurnAdj(1,-(speed),0xffff);
}
```
