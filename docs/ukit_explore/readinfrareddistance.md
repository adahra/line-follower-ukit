> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/infrared/readinfrareddistance.md).

# readInfraredDistance(id)

Get infrared sensor distance value.

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Return Values

| Parameters | Type | Description               |
| ---------- | ---- | ------------------------- |
| number     | int  | Distance, range 0 \~ 20cm |

### Example

Use the uKit Explore v2 board to display the distance read by infrared sensor ID-1 every 400ms

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    Serial.println(readInfraredDistance(1));    
    delay(400);
}
```

Use the uKit Explore v2 board to read the infrared sensor distance. When the distance is less than 5cm, set the on-board LED to RED, and when the distance is more than 5cm, set the on-board LED to GREEN.

```c
#include "uKitExplore2.h"
volatile int item;

void setup() {    
    Initialization();     
    item = 0;
}

void loop() {    
    item=readInfraredDistance(1));    
    if(item>0 && item<5){        
        setRgbledColor(255,0,0);        
        delay(300);      
    }    
    else{        
        setRgbledColor(0,255,0);        
        delay(300);          
    }
}
```
