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
