# readButtonValue(id)

Get the state of the touch sensor

### Parameter

| Parameters | Type          | Description                               |
| ---------- | ------------- | ----------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10 |

### Return Values

| Parameters | Type | Description                                                                                         |
| ---------- | ---- | --------------------------------------------------------------------------------------------------- |
| number     | int  | <p>State of touch sensor: </p><p>1 – Single press </p><p>2 – Double press </p><p>3 – Long press</p> |

### Example

Use the uKit Explore v2 board to read the state of the touch sensor. When single press, set the on-board LED to RED for 400ms. When double press, set the on-board LED to GREEN for 400ms. When long press, set the on-board LED to BLUE for 400ms. The on-board LED is off when not press.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {    
    int ButtonState1 = 0;    
    ButtonState1 = readButtonValue(1);    
    if(ButtonState1  == 1){        
        setRgbledColor(255,0,0);        
        delay(400);    
    }    
    else if(ButtonState1  == 2){ 
        setRgbledColor(0,255,0);        
        delay(400);    
    }    
    else if(ButtonState1  == 3){        
        setRgbledColor(0,0,255);        
        delay(400);    
    }    
    else{    
        setRgbledColor(0,0,0);    
    }    
}

```
