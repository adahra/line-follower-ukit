# IMU::init()

Gyroscope initialization function

### Example

{% hint style="info" %}
This feature requires an upgrade to the latest [SDK](https://ubtechedu.gitbook.io/ukit-explore/ukit-explore-quick-start/get-sdk).
{% endhint %}

Use the uKit Explore v2 board to get the XYZ acceleration of the gyroscope every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    IMU::init();
}

void loop() {    
    IMU::read();    
    Serial.print("xAccel:");    
    Serial.print(IMU::getRawAccelX());    
    Serial.print(",yAccel:");    
    Serial.print(IMU::getRawAccelY());    
    Serial.print(",zAccel:");    
    Serial.println(IMU::getRawAccelZ());    
    delay(400);
}
```
