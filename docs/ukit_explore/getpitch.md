# IMU::getPitch()

Obtain the pitch angle θ (pitch): rotate around the Y axis.

### Return Values

| Parameters | Type | Description |
| ---------- | ---- | ----------- |

| number | float | Pitch angle θ (pitch): rotating around the Y axis |
| ------ | ----- | ------------------------------------------------- |

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

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5UTuA5HigfsS9sVGKj%2F-M5UV9h4w5hmUFGccApA%2Fimage.png?alt=media\&token=4aff8983-8f55-4f7c-b525-fca84dbfedc3)

![Obtain the pitch angle θ (pitch): rotate around the Y axis.](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpS-QFq6IG-_5PH3SI-%2F-LpS2-8da5MucV5YmDBd%2F2.gif?alt=media\&token=d97c990e-a13c-444f-9d69-523cf71072d1)
