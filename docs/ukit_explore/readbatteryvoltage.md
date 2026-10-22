> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/onborad/onboard-battery-voltage-level/readbatteryvoltage.md).

# readBatteryVoltage()

Get battery voltage

### Return Values

| Parameters | Type  | Description                    |
| ---------- | ----- | ------------------------------ |
| number     | float | Voltage value, range 0 \~ 8.4V |

### Example

Use uKit Explore v2 board, connect the battery and servo ID-1, and then turn on the switch. When the battery voltage is less than 7.2V, set the buzzer to give alarm, set the RGB LED to red light and stop the servo motor, the program no longer runs. If it is greater than 7.2V, the servo continues to rotate.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    if(readBatteryVoltage()<=7.2){	
        setServoStop(1);	
        tone(131,400);	
        setRgbledColor(255,0,0);	
        while(1);    
    }    
    else{	
        setServoTurn(1,0,80);
    }
}
```
