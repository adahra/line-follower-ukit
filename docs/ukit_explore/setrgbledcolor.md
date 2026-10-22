# setRgbledColor(red,green,blue)

Set onboard RGB lights to different colors.

### Parameter

| Parameters | Type | Description                                               |
| ---------- | ---- | --------------------------------------------------------- |
| red        | int  | pwm set RGB light, red light brightness: range 0 \~ 255   |
| green      | int  | pwm set RGB light, green light brightness: range 0 \~ 255 |
| blue       | int  | pwm set RGB light, blue light brightness: range 0 \~ 255  |

### Example

Use the uKit Explore v2 board to control the onboard RGB LED to turn red, and then turn to purple after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setRgbledColor(255,0,0);    
    delay(400);    
    setRgbledColor(255,0,255);
}

void loop() { 

 }
```

{% hint style="info" %}
About brightness control: As long as the mixture color ratio remains the same, the brightness will be decreased.
{% endhint %}

Use the uKit Explore v2 board to the onboard RGB LED to turn purple (brightest), to purple (medium intensity) after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setRgbledColor(255,0,255);    
    delay(400);    
    setRgbledColor(127,0,127);
}

void loop() { 
   }
```

Use the uKit Explore v2 board to the onboard RGB LED to turn purple (brightest), to purple (medium intensity) after 400ms and turn off after another 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setRgbledColor(255,0,255);    
    delay(400);    
    setRgbledColor(127,0,127);    
    delay(400);     
    setRgbledColor(0,0,0);
}

void loop() {
    }
```
