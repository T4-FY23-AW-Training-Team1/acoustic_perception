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
uint16 activity
float64 max_spectrum
float32 unit_direction_x
float32 unit_direction_y
float32 duration_time
```
`unit_direction_x` and `unit_direction_y` are the unit direction vector.

`activity` is a binary variable whether the sound is active or not, and `duration time` counts how many seconds does it long.

`max_spectrum` shows the maximum MUSIC spectrum, which can be said that the spectrum of the estimated direction.

## sound_source_localizer
This package collects sound source directions from the microphone array, and converts them as `SoundSourceDirection.msg` type topics and publish.

Parameters can be set in `sound_source_localizer.param.yaml`

A `geometry_msgs/marker` type topic is also published as an arrow, showing the estimated direction and activity.