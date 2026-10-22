# setEyelightAllPetals(id,red,green,blue)

Control all the 8 petals (pixels) of the eye lamp to light up a set of color.

### Parameter

| Parameters | Type          | Description                                               |
| ---------- | ------------- | --------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                 |
| red        | int           | pwm set RGB light, red light brightness: range 0 \~ 255   |
| green      | int           | pwm set RGB light, green light brightness: range 0 \~ 255 |
| blue       | int           | pwm set RGB light, blue light brightness: range 0 \~ 255  |

### Example:

Use the uKit Explore v2 board to control all the petals of eye lamp ID-1 to turn red, and turn to purple after 400ms.

```c
#include "uKitExplore2.h"

void setup() {    
    Initialization();    
    setEyelightAllPetals(1,255,0,0);    
    delay(400);    
    setEyelightAllPetals(1,255,0,255);
}

void loop() {

}
```

{% hint style="info" %}
About brightness control: As long as the mixture color ratio remains the same, the brightness will be decreased.
{% endhint %}

Use the uKit Explore v2 board to control all the petals of the eye lamp to be purple (brightest), and after 400ms to purple (medium intensity).

```c
#include "uKitExplore2.h"

void setup() {    
    Initialization();    
    setEyelightAllPetals(1,255,0,255);    
    delay(400);    
    setEyelightAllPetals(1,127,0,127);
}

void loop() {

}
```
