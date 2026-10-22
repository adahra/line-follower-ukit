> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/onborad/buzzer/tone.md).

# tone(frequency, duration)

Set the onboard buzzer to sound.

### Parameter

| Parameters | Type          | Description                                 |
| ---------- | ------------- | ------------------------------------------- |
| frequency  | unsigned int  | Frequency at which sound is produced, in HZ |
| duration   | unsigned long | The duration of the sound, in milliseconds  |

### Example

Use the uKit Explore v2 board to control the onboard buzzer to play sound 131Hz for 400ms

```c
#include "uKitExplore2.h"
void setup() {
    Initialization();
    tone(131,400);
}
void loop() {
    
}
```
