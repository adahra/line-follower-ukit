> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/ultrasonic/setultrasonicrgbledoff.md).

# setUltrasonicRgbledOff(id)

Turn off the ultrasonic sensor’s LED light.

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Example

Use the uKit Explore v2 board to control the ultrasonic sensor ID-1’s LED to red, and turn off after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setUltrasonicRgbledOff(1,255,0,0);    
    delay(400);    
    setEyelightOff(1);
}

void loop() {
}
```
