> For the complete documentation index, see [llms.txt](https://ubtechedu.gitbook.io/ukit-explore/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://ubtechedu.gitbook.io/ukit-explore/standard/sensor/color/readcolor.md).

# readColor(id,color)

Recognize the specific color from the color sensor

### Parameter

| Parameters | Type          | Description                                                                                                              |
| ---------- | ------------- | ------------------------------------------------------------------------------------------------------------------------ |
| id         | unsigned char | ID number to be controlled, range 0 \~ 10                                                                                |
| color      | String        | Key in the color to be detected. Currently supported: Red, Green, Blue, Yellow, Cyan, Purple, Orange, Black, White, Gray |

### Return Values

| Parameters | Type    | Description                                                     |
| ---------- | ------- | --------------------------------------------------------------- |
| number     | boolean | Return True if the set color is the same as the detected color. |

### Example
