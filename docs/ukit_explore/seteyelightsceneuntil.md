> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/eyelight/seteyelightsceneuntil.md).

# setEyelightSceneUntil(id,scene,times)

Set the eye lamp to the different scene mode (blocking).

### Parameter

| Parameters | Type          | Description                                                                                                                 |
| ---------- | ------------- | --------------------------------------------------------------------------------------------------------------------------- |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                   |
| scene      | char          | <p>Scene types</p><p>0 colorful horse race lamp, </p><p>1 disco, </p><p>2 three primary colors, </p><p>3 color stacking</p> |
| times      | int           | Emoticons repeats how many times                                                                                            |

### Example

{% hint style="info" %}
This feature requires an upgrade to the latest [SDK](https://ubtechedu.gitbook.io/ukit-explore/ukit-explore-quick-start/get-sdk).
{% endhint %}

Use the uKit Explore v2 board to control the eye lamp ID-1 to display colorful horse race lamp and repeat 5 times,

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightSceneUntil(1,0,5);
}

void loop() {

}
```

Use the uKit Explore v2 board to control the eye lamp ID-1 to display colorful horse race lamp, repeat 5 times, and then the eye lamp ID-2 to display colorful horse race lamp, repeat 5 times.

```c
#include "uKitExplore2.h"
void setup() {    
    Initialization();    
    setEyelightSceneUntil(1,0,5);    
    setEyelightSceneUntil(2,0,5);
}

void loop() {

}
```
