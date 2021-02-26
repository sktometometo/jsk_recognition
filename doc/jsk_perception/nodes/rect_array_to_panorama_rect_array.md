# rect_array_to_image_marker.py

Convert `jsk_recognition_msgs/RectArray` to `jsk_recognition_msgs/PanoramaRectArray`.

## Subscribing Topics
* `~input` (`jsk_recognition_msgs/RectArray`)
- `~input_panorama_info` (`jsk_recognition_msgs/PanoramaInfo`)
- `~input_panorama_image` (`sensor_msgs/Image`)

## Publishing Topics
* `~output` (`jsk_recognition_msgs/PanoramaRectArray`)