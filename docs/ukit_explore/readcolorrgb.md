> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/color/readcolorrgb.md).

# readColorRgb(id)

Get the RGB value of color sensor.

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Return Values

| Parameters | Type             | Description                                                                                       |
| ---------- | ---------------- | ------------------------------------------------------------------------------------------------- |
| array      | unsigned char \* | Return an RGB array. The first is the R value, the second is the G value and the third is B value |

### Example

Use the uKit Explore v2 board to display the RGB value read from color sensor every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
Initialization();
}

void loop() {    
    unsigned char *rgbValues=NULL;    
    rgbValues=readColorRgb(1);    
    Serial.println("R value:");    
    Serial.println(rgbValues[0]);    
    Serial.println(",G value:");    
    Serial.println(rgbValues[1]);    
    Serial.println(",B value:");    
    Serial.println(rgbValues[2]);    
    delay(400);
}
```

Use the uKit Explore v2 on-board RGB LED to show the color read by the color sensor.

```c
#include "uKitExplore2.h"
unsigned char *rgbValues=NULL;
void setup() {    
    Initialization();
}

void loop() {    
    rgbValues=readColorRgb(1);    
    setRgbledColor(rgbValues[0],rgbValues[1],rgbValues[2]);   
    delay(100);
}
```
