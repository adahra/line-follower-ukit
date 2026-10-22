# button1.Update();

Update onboard button status

### Example

Use the uKit Explore v2 board, when the onboard button is clicked, the red light is on for 400ms; When double-clicked, the green light is on for 400ms, and when long-pressed, the blue light is on for 400ms. The light is off when button not pressed.

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
