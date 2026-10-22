# IMU::getRoll()

Get the roll angle Φ (roll): the angle of rotation around the X axis.

### Return Values

| Parameters | Type | Description |
| ---------- | ---- | ----------- |

| number | float | Roll angle Φ (roll): the angle of rotation around the X axis |
| ------ | ----- | ------------------------------------------------------------ |

### Example

This feature requires an upgrade to the latest [SDK](get-sdk.md).

Use the uKit Explore v2 board to get the roll and pitch angle of the gyroscope and display it every 400ms to the serial monitor.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    IMU::init();
}

void loop() {    
    IMU::read();    
    Serial.print("Roll:");    
    Serial.print(IMU::getRoll());    
    Serial.print(",Pitch:");    
    Serial.println(IMU::getPitch());    
    delay(400);
}
```

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5UVF4PQijU8tjzBfW3%2F-M5UVGn-xPUPki6oxKHt%2Fimage.png?alt=media\&token=3f03540a-d3d1-4e92-bb97-ac92b455df43)

![Get the roll angle Φ (roll): the angle of rotation around the X axis.](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpS-QFq6IG-_5PH3SI-%2F-LpS1W2I-8lYz4aLReys%2F1.gif?alt=media\&token=9afb465d-914f-4f01-ab06-5d76581beef1)
