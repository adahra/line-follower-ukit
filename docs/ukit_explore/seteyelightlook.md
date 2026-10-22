> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/eyelight/seteyelightlook.md).

# setEyelightLook(id,face,times,red,green,blue)

Set the emoticons to light up different color of expressions (non-blocking)

### Parameter

| Parameters | Type          | Description                                                                                                                                                                                                                                                  |
| ---------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                                                                                                                                                    |
| face       | int           | <p>face is the type of expression </p><p>0 blinks, </p><p>1 sad, </p><p>2 hot tears, </p><p>3 flashing tears, </p><p>4 crying, </p><p>5 dizzy, </p><p>6 happy, </p><p>7 surprised, </p><p>8 breathing, </p><p>9 blinking, </p><p>10 fan, </p><p>11 wiper</p> |
| times      | int           | Emoticons repeats how many times                                                                                                                                                                                                                             |
| red        | int           | pwm set RGB light, red light brightness: range 0 \~ 255                                                                                                                                                                                                      |
| green      | int           | pwm set RGB light, green light brightness: range 0 \~ 255                                                                                                                                                                                                    |
| blue       | int           | pwm set RGB light, blue light brightness: range 0 \~ 255                                                                                                                                                                                                     |

### Example

Use the uKit Explore v2 board to control the blinking of eye lamp ID-1 in red and 5 times

```c
#include "uKitExplore2.h"

void setup() {    
    Initialization();    
    setEyelightLook(1,0,5,254,0,0);
}

void loop() {

}
```

Use the uKit Explore v2 board to control the blinking of eye lamp ID-1, ID-2 in red and 5 times

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightLook(1,0,5,254,0,0);    
    setEyelightLook(2,0,5,254,0,0);
}

void loop() {

}
```
