# setUltrasonicRgbled(id,red,green,blue)

Control the ultrasonic sensor’s LED to light up and set the color

### Parameter

| Parameters | Type          | Description                                               |
| ---------- | ------------- | --------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                 |
| red        | int           | pwm set RGB light, red light brightness: range 0 \~ 255   |
| green      | int           | pwm set RGB light, green light brightness: range 0 \~ 255 |
| blue       | int           | pwm set RGB light, blue light brightness: range 0 \~ 255  |

### Example

Use uKit Explore v2 board to control the ultrasonic sensor ID-1’s LED to red, and to purple after 400ms

```c
#include "uKitExplore2.h"

void setup() {    
    Initialization();    
    setUltrasonicRgbled(1,255,0,0);    
    delay(400);    
    setUltrasonicRgbled(1,255,0,255);
}

void loop() {
}
```

{% hint style="info" %}
About brightness control: If the mixture color ratio remains the same, the brightness will be decreased.
{% endhint %}

Use uKit Explore v2 board to control the ultrasonic sensor ID-1’s LED to red (brightest) and to purple (medium intensity) after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setUltrasonicRgbled(1,255,0,255);    
    delay(400);    
    setUltrasonicRgbled(1,127,0,127);
}
void loop() {
}
```
