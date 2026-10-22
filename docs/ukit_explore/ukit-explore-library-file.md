# Introduction to uKit Explore Library

uKit Explore main controller is the development board designed by UBTECH for uKit product series. It is based on Arduino Mega 2560 and can be used to control uKit’s electronic components such as servo and DC motors.&#x20;

On the uKit Explore main controller, there is a buzzer, RGB LED, programmable button, gyroscope and others rich expansion interfaces. It can be powered by USB or 2-S Lipo battery (7.4V), and support on-board charging of the li-po battery.&#x20;

Furthermore, uKit Explore controller board consists of rich peripherals interfaces such as ADC, I2C, SPI and UART and its programming is compatible with Arduino.

### 1、About uKit Explore Library

The hardware of uKit Explore is divided into v1 and v2. The pinout for both version is slight difference. If the board is v1, then use the uKitExplore.h library. If the board is v2, then use the uKitExplore2.h&#x20;

The uKit Explore library supports languages such as Chinese, English, Korean, Thai and Russia.

```c
/*V1*/
#include "uKitExplore.h"
#include "uKitExploreEn.h"
#include "uKitExploreKo.h"
#include "uKitExploreTh.h"
#include "uKitExploreRu.h"
/*V2*/
#include "uKitExplore2.h"
#include "uKitExplore2En.h"
#include "uKitExplore2Ko.h"
#include "uKitExplore2Th.h"
#include "uKitExplore2Ru.h"
```

Library file included all the uKit Explore hardware controls. Hence, when using the uKit Explore library, the user needs to add Initialization() on setup code.

```c
#include "uKitExplore.h"
void setup(){
    Initialization();//初始化硬件
}

void loop() {
}
```

### 2、About ucode library

Ucode library is simpler as compared to uKit Explore library. No matter you are using v1 or v2 of the uKit Explore controller board, you only need to add the ucode library to get it working.

During initialization, the library will detect the hardware by itself and then automatically initialize the corresponding hardware. At the same time, the ucode library can also control all the hardware of uKit Explore.

Since the ucode library retains the communication layer, there is a fixed format when using it as follows:

```c
#include "ucode.h"
void setup() {    
    Initialization();    
    if (protocolRunState == false) {  
     
     }
}
void loop() {
    protocol();    
    if (protocolRunState == false) {
    
    }
}
```

Of course, if you just want to do C programming, and do not mind you will be prompted to re-flash the firmware on the next time you connect to uCode software, you can also use the following format:

```c
#include "ucode.h"
void setup() {
    Initialization();
}
void loop() {

}
```
