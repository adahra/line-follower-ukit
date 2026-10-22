> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/eyelight/seteyelightlookuntil.md).

# setEyelightLookUntil(id,face,times,red,green,blue)

设置眼灯亮不同颜色的表情（阻塞）

### Parameter

| Parameters | Type          | Description                                                                                                                                                                                                                                                  |
| ---------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                                                                                                                                                    |
| face       | int           | <p>face is the type of expression </p><p>0 blinks, </p><p>1 sad, </p><p>2 hot tears, </p><p>3 flashing tears, </p><p>4 crying, </p><p>5 dizzy, </p><p>6 happy, </p><p>7 surprised, </p><p>8 breathing, </p><p>9 blinking, </p><p>10 fan, </p><p>11 wiper</p> |
| times      | int           | Emoticons repeats how many times                                                                                                                                                                                                                             |
| red        | int           | pwm set RGB light, red light brightness: range 0 \~ 255                                                                                                                                                                                                      |
| green      | int           | pwm set RGB light, green light brightness: range 0 \~ 255                                                                                                                                                                                                    |
| blue       | int           | pwm set RGB light, blue light brightness: range 0 \~ 255                                                                                                                                                                                                     |

### 例子

{% hint style="info" %}
This feature requires an upgrade to the latest [SDK](https://ubtechedu.gitbook.io/ukit-explore/ukit-explore-quick-start/get-sdk).
{% endhint %}

Use the uKit Explore v2 board to control the blinking of eye lamp ID-1 in red and 5 times

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightLookUntil(1,0,5,254,0,0);
}

void loop() {
}
```

Use the uKit Explore v2 board to simultaneously control the blinking of the eye lamp ID-1 in red and 5 times, and then the blinking of the eye lamp ID-2, in red and 5 times:

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightLookUntil(1,0,5,254,0,0);    
    setEyelightLookUntil(2,0,5,254,0,0);
}
void loop() {

}
```
