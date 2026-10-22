> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/onborad/buzzer/notone.md).

# noTone(pin)

Set the onboard buzzer to stop playing

### Parameter

| Parameters | Type          | Description                                             |
| ---------- | ------------- | ------------------------------------------------------- |
| pin        | unsigned char | Buzzer pin, uKit Explore v1 is 43, uKit Explorev2 is 38 |

### Example

Use the uKit Explore v2 board to control the onboard buzzer to continuously emit 131Hz sound for 400ms and turn off the buzzer for 400ms.

```c
#include "uKitExplore2.h"
void setup() {
    Initialization();

}
void loop() {
    tone(131,400);
    noTone(43);
    delay(400);
    
}
```
