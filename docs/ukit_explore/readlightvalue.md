# readLightValue(id)

Get the light value from the light sensor.

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Return Values

| Parameters | Type | Description                      |
| ---------- | ---- | -------------------------------- |
| number     | int  | Brightness Value (0 \~ 4000 lux) |

### Example

Use the uKit Explore v2 board to display the light value by light sensor ID-1 every 400ms

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    Serial.println(readLightValue(1));    
    delay(400);
}
```
