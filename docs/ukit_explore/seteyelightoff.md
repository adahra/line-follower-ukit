> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/eyelight/seteyelightoff.md).

# setEyelightOff(id)

Turn off the eye lamp

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

Use the uKit Explore v2 board to control all the petals (pixels) of the eye lamp ID-1 to turn RED, and turn off after 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightAllPetals(1,255,0,0);    
    delay(400);    
    setEyelightOff(1);
    }
void loop() {

}
```
