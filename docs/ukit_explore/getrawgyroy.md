> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/onborad/imu/getrawgyroy.md).

# IMU::getRawGyroY()

Get the gyroscope's Y-axis angular velocity

### Return Values

| Parameters | Type | Description |
| ---------- | ---- | ----------- |

| number | float | Y-axis angular velocity °/s |
| ------ | ----- | --------------------------- |

### Example

{% hint style="info" %}
This feature requires an upgrade to the latest [SDK](https://ubtechedu.gitbook.io/ukit-explore/ukit-explore-quick-start/get-sdk).
{% endhint %}

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
