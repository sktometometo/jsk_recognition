# panorama_rect_array_to_bounding_box_array.py

Convert `jsk_recognition_msgs/PanoramaRectArray` with `jsk_recognition_msgs/ClassificationResult` to `BoundingBoxArray`.

## Subscribing Topics
* `~input_class` (`jsk_recognition_msgs/ClassificationResult`)
- `~input_panorama_rects` (`jsk_recognition_msgs/PanoramaRectArray`)

## Publishing Topics
* `~output` (`jsk_recognition_msgs/BoundingBoxArray`)

## Parameters
* `~frame_fixed` (`string`, default: `fixed_frame`)

frame name fixed to the world coordinate. BoundingBoxArray messages are published with this frame

* `~dimensions` (`dict of list with label as keys`, default: `None`)

dimensions of bounding box for each class. z field is also used for calculation of distance.

Examples:
```
0: [1.0, 1.0, 1.0]
1: [0.5, 0.5. 2.0]
```
