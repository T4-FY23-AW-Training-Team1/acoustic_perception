# acoustic_perception
This repository includes packages which are useful for using microphone arrays.

The microphone array [ReSpeaker ver 2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) is considered here.

Try [here](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/#update-firmware) if you cannot connect to ReSpeaker.

The following is a description of the packages.

## acoustics_msgs
This is a package that contains a message type `SoundSourceDirection.msg` 

The format of `SoundSourceDirection.msg` looks like this.

```
std_msgs/Header header
visualization_msgs/Marker direction_arrow
uint16 activity
float32 unit_direction_x
float32 unit_direction_y
float32 duration_time
```
`unit_direction_x` and `unit_direction_y` are the unit direction vector.

`activity` is a binary variable whether the sound is active or not, and `duration time` counts how many seconds does it long.

`direction_arrow` can be used for debugging

## sound_source_localizer
This package collects sound source directions from the microphone array, and converts them as `SoundSourceDirection.msg` type topics and publish.

The period of collecting and publishing can be determined as a parameter in `create_wall_timer`. (Default=20ms)