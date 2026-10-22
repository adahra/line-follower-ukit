> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/eyelight/seteyelightpetals.md).

# setEyelightPetals(id,petalsnum,petals)

Customize eye lamps, 8 petals (pixels) light up in different colors.

### Parameter

| Parameters | Type          | Description                                                                                                                                          |
| ---------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                                            |
| petalsnum  | int           | Number of light petals (pixels) to be lit, range 0 \~ 8, default is 8                                                                                |
| petals     | int           | Set the color of each light petal. Fill in a fixed Json format {“data” \[first lamp petal number, r, g, b, second lamp petal number, r, g, b, ....]} |

### Json

{“Data” \[First lamp petal number, r, g, b, second lamp petal number, r, g, b, ..., eighth lamp petal number, r, g, b ,]} Each light bulb of the eye light has a fixed number.

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QpfPCsNt8FosEHLoq%2F-M5QqcS0kwvNy0wAh1ik%2Fimage.png?alt=media\&token=e02b0d22-fc05-40c0-9717-21207dcaeaf2)

### Example

Use the ukitexplore V2 board to control the ID-1 eye light to be on, as shown in the above figure:

```c
#include "uKitExplore2.h"

String colours_json;

void setup() {    
    Initialization();
   
}

void loop() {    
    colours_json = "{\"data\":\
        [1, 0, 0, 0,\        
        2, 255, 255, 255,\        
        4, 255, 240, 0,\        
        8, 0, 255, 0,\        
        16, 255, 0, 0,\        
        32, 0, 0, 255,\        
        64, 255, 0, 255,\        
        128, 255, 255, 255]}";    
    setEyelightPetals(1, 8, colours_json);
}
```
