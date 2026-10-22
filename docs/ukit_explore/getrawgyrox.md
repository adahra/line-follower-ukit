# IMU::getRawGyroX()

Get the gyroscope's X-axis angular velocity

### Return Values

| Parameters | Type | Description |
| ---------- | ---- | ----------- |

| number | float | X-axis angular velocity °/s |
| ------ | ----- | --------------------------- |

### Example

This feature requires an upgrade to the latest [SDK](get-sdk.md).

Use uKit Explore v2 board to get the XYZ angular velocity of the gyroscope and display it every 400ms to the serial monitor.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    IMU::init();
}

void loop() {    
    IMU::read();    
    Serial.print("xGyro:");    
    Serial.print(IMU::getRawGyroX());    
    Serial.print(",yGyro:");    
    Serial.print(IMU::getRawGyroY());    
    Serial.print(",zGyro:");    
    Serial.println(IMU::getRawGyroZ());    
    delay(400);
}
```
