> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/ukit-explore-quick-start/hui-du-chuan-gan-qi-gu-jian-geng-xin.md).

# Grayscale Sensor Firmware Update

### 1. Download the grayscale sensor firmware

&#x20;     a. [V1.1 grayscale sensor firmware](https://link.zhihu.com/?target=https%3A//github.com/UBTEDU/Patrol_Sensor/releases/download/3.5/route3.5.zip)

&#x20;     b. [V1.0 grayscale sensor firmware](https://link.zhihu.com/?target=https%3A//github.com/UBTEDU/Patrol_Sensor/releases/download/3.3/route3.3.zip)

{% hint style="info" %}
Note: V1.0 Grayscale Sensor firmware to be used with V1.0 Grayscale Sensor hardware and V1.1 Grayscale Sensor firmware to be used with V1.1 Grayscale Sensor hardware.
{% endhint %}

### 2. Install [uKit Explore Blockly](https://www.ubtechedu.com/global/Uploads/file/20190221/uKit-Explore_Setup.zip)

&#x20;

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QkWpeKXYDSIcrRUpM%2F-M5Qn7DVMkgwjPGrsLd_%2Fimage.png?alt=media\&token=b56e91e9-c316-4e5d-b651-f2961bf0c191)

### 3. Unzip the downloaded file routex.x.zip, and open routex.x.ino

<div align="left"><img src="https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QkWpeKXYDSIcrRUpM%2F-M5QnHuKNOTVG20OE8np%2Fimage.png?alt=media&amp;token=0df42bb9-c952-4f3b-9dad-e669e232d9a4" alt=""></div>

### 4. Connect the USB to the USB port on Grayscale sensor, and another end to your PC. (make sure the USB is of data cable instead of charging cable)

### 5. Choose the correct board and COM Port

* **5.1** Board: Tools > Board > Arduino Nano
* **5.2** Processor: If your Arduino IDE version is above 1.8.6, you can choose the default ATmega328P. If the Arduino IDE version is lower than 1.8.6, select ATmega328P (Old Bootloader)
* **5.3** COM Port: Select your COM port (you might have different port)

![](https://3911929090-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-Lp2wlXBWybR4t3JbPbo%2F-M5QkWpeKXYDSIcrRUpM%2F-M5QnotRZyqvlvNxFnHE%2Fimage.png?alt=media\&token=0a3a1f4b-9177-4007-8f1a-86e3b56fbd0a)

### **6.**&#x43;lick upload and upon successful upload, the grayscale sensor’s firmware had been updated.
