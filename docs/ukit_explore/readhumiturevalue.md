# readHumitureValue(id,choice)

Get the temperature or humidity value

### Parameter

| Parameters | Type          | Description                                                                                                                                               |
| ---------- | ------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                                                 |
| choice     | char          | <p>Key in character for your options. </p><p>C – Obtain temperature in Celsius </p><p>F – Obtain temperature in Fahrenheit </p><p>H – Obtain humidity</p> |

### Return Values

| Parameters | Type | Description                    |
| ---------- | ---- | ------------------------------ |
| number     | int  | Return the corresponding value |

### Example

Use the uKit Explore v2 board to display the temperature value in Celsius & Fahrenheit and humidity value every 400ms.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();
}

void loop() {       
    Serial.print("Temperature:");    
    Serial.print(readHumitureValue(1,'C'));    
    Serial.print(" °C");    
    Serial.print(",Temperature:");    
    Serial.print(readHumitureValue(1,'F'));    
    Serial.print(" °F");    
    Serial.print(",Humidity:");    
    Serial.println(readHumitureValue(1,'H'));    
    delay(400);
}
```
