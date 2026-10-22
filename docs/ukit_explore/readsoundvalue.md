# readSoundValue(id)

Get the sound intensity.

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Return Values

| Parameters | Type | Description                 |
| ---------- | ---- | --------------------------- |
| number     | int  | Sound intensity (0 \~ 1023) |

### Example

Use the uKit Explore v2 board to display the sound intensity by sound sensor ID-1 every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    Serial.println(readSoundValue(1));    
    delay(400);
}
```
