# MCS Research Project - Mixed Reality for Robotics



## [Devices](docs/devices.md)



## Real-time XR Object Detection

#### **Casting Meta Quest 3 View**

- Developer Hub - Beta Cast 2.0
- Add the casting window to [OBS](https://obsproject.com/) sources
- Stream settings
  - Server: `rtmp://127.0.0.1`
  - Set an arbitrary **Stream Key**
- Use [MonaServer](https://www.monaserver.ovh/) for local RTMP streaming
  - `rtmp://127.0.0.1//<Stream Key>`
- Output Settings
  - Video Bitrate: 1800 Kbps
  - Audio Bitrate: 64 (min)
  - Encoder Preset: Fastest
- Video Settings
  - Resolution: 1280 * 1280
  - FPS: 30



#### [YOLOv8](https://docs.ultralytics.com/)

[**GitHub**](https://github.com/ultralytics/ultralytics)

##### Resources

- YouTube
  - [Ultralytics YOLOv8](https://www.youtube.com/playlist?list=PL1FZnkj4ad1PFJTjW4mWpHZhzgJinkNV0)
  - [YOLOv8 Tutorial](https://www.youtube.com/playlist?list=PLZCA39VpuaZZ1cjH4vEIdXIb0dCpZs3Y5)
- Colab
  - [YOLOv8 Tutorial](https://colab.research.google.com/github/ultralytics/ultralytics/blob/main/examples/tutorial.ipynb#scrollTo=ZY2VXXXu74w5)
  - [How to Train YOLOv8 Object Detection on a Custom Dataset](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/train-yolov8-object-detection-on-custom-dataset.ipynb)





## Unity

#### Config

- Editor Version: `2022.3.19f1`
- Packages
  - [Meta XR All-in-One SDK](https://assetstore.unity.com/packages/tools/integration/meta-xr-all-in-one-sdk-269657)
  - [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)



#### Interactions

- The user can grab and place two virtual spheres of different colors, overlaying them on the table to represent two categories.





## LFD

* Detect object's location and pose
  * Depth Camera
  * Tag
* Collect trajectory data
* Save as hdf5
* Training

* Run the policy





## Academic Outputs

### [Proposal](docs/proposal.pdf)
