# How to Install uKit Explore Arduino Library

### Step 1: Download Arduino IDE

Below is the example for the Windows users. You can download the Arduino IE from the official Arduino website: [Arduino IDE ](http://arduino.cc/en/Main/Software).

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpWDlq8FpB6AVpNqpsM%2F-LpWE-Sn68wLA203--RB%2Fimage.png?alt=media\&token=888f39ce-3595-4ead-9d41-d336f026b0e8)

Recommended to use at least Arduino 1.8.0 or above.

Click on the Windows Installers to download it, and then follow the instructions to install the Arduino IDE on your laptop.

&#x20;

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpWDlq8FpB6AVpNqpsM%2F-LpWEDG4NMfdGSVhkHG3%2Fimage.png?alt=media\&token=5fb71ee0-2b9e-4e9c-8aa9-decfc90a3750)

### Step 2: Download uKit Explore Arduino Library

Browse to <https://github.com/UBTEDU/uKitExplore-library>, and click on the Clone or download button to Download ZIP.

&#x20;

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpWDlq8FpB6AVpNqpsM%2F-LpWER-qtT2g4pS_JcHK%2Fimage.png?alt=media\&token=4f3e53a0-efa5-4a3b-b52b-caf66dd878cc)

### Step 3: Add the uKit Explore Library into Arduino IDE

Open the Arduino IDE, go to Sketch > Include Library > Add .ZIP library&#x20;

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-LpWDlq8FpB6AVpNqpsM%2F-LpWEWVOC9IOFhsW8pVT%2Fimage.png?alt=media\&token=4b27633e-e8e1-435a-b93a-c84421fb693c)

and browse to the location of the downloaded ZIP file in Step 2 to add the library.

<div align="left"><img src="https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QfQy2kYjzUtlo4I7J%2F-M5QgsP7am9sj5JNzlJd%2Fimage.png?alt=media&amp;token=81a68094-9877-4eb3-968b-3361a8a89783" alt=""></div>

### Step 4：Verify the installation of uKit Explore Library

Open the Arduino IDE, copy and paste the below code.

&#x20;Make sure you selected the board as **Arduino Mega or Mega 2560**, and correct COM port.

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QfQy2kYjzUtlo4I7J%2F-M5Qh-o_7SFddyHYG96K%2Fimage.png?alt=media\&token=1328b4ce-751e-462e-9273-f0c4c8a718de)

If it able to compile and upload, the installation of the uKit Explore library is successful!

```c
#include "uKitExplore2.h"
void setup() {
    Initialization();
}
void loop() {
}
```
