> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/motor/setmotorturn.md).

# setMotorTurn(id,pwmDuty)

Set the rotation speed of the DC motor using PWM.

### Parameter

| Parameters | Type          | Description                                                                                                                                                |
| ---------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 18                                                                                                                  |
| speed      | int           | Set motor rotation speed. Positive numbers indicate clockwise rotation, negative numbers indicate counterclockwise rotation, unit: pwm, range: -140 \~ 140 |

###

{% hint style="info" %}
About the difference between speed and PWM mode: speed mode is a closed-loop control that allows the motor to maintain a certain speed. The disadvantage: forward and backward switching requires 100ms waiting. The PWM mode is simply to control the rotation speed with voltage, and the forward and backward rotation can be switched without waiting. Disadvantages: The speed will be changed by the weight of the car.
{% endhint %}

### Example

Use uKit Explore v2 board to control the DC Motor ID-1 to rotate counterclockwise all the time with the PWM is 80

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setMotorTurn(1,-80); 
}
    
void loop() {
    }
```

Use the uKit Explore v2 board to control the DC Motor ID-1 to rotate counterclockwise all the time with the PWM slowly increases from 0 to the maximum speed.

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
    setMotorTurn(1,-(speed));
}
```
