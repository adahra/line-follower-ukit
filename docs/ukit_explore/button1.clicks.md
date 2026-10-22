> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/onborad/button/button1.clicks.md).

# button1.clicks()

Get the onboard button status

### Return Values

| Parameters | Type | Description                                                                                 |
| ---------- | ---- | ------------------------------------------------------------------------------------------- |
| number     | int  | <p>return value </p><p>1 – Single press </p><p>2- Double press </p><p>-1 – Long pressed</p> |

### Example

It is controlled by ukitexplore V2 board. When the on-board button is clicked, the red light will be on for 400ms; when the button is double clicked, the green light will be on for 400ms, the long-term time will be on, and the blue light will be on for 400ms. The light is off without pressing.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    button1.Update();    
    if(button1.clicks == 1){        
        setRgbledColor(255,0,0);        
        delay(400);    
    }    
    else if(button1.clicks == 2){        
        setRgbledColor(0,255,0);        
        delay(400);    
    }    
    else if(button1.clicks == -1){        
        setRgbledColor(0,0,255);        
        delay(400);    
    }    
    else{    
        setRgbledColor(0,0,0);
    }    
}
```
