# migrave_audio_recorder
Repository for an audio recorder used in the MigrAVE project

The package contains two nodes:
- migrave_audio_recorder
    - Audio recorder for QTrobot's internal [Respeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) which requires the [`qt_respeaker_app`](https://github.com/luxai-qtrobot/software/tree/master/apps/qt_respeaker_app) node.
- migrave_audio_rode_recorder
    - Audio recorder for the external RØDE NTG microphone

## Depedencies

- `qt_respeaker_app`
- `audio_common`

## Usage 

Launch the two recorder nodes:
```sh
roslaunch migrave_audio_recorder migrave_audio_recorder.launch
```
Laucn the ROS driver for the RØDE NTG microphone (if it is connected)
```sh
roslaunch migrave_audio_recorder usb_microphone.launch
```
Here the RØDE microphone is assumed to be listed as device `hw:1,0` (use `arecord -l` to check).

Start recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "True"
```
Stop recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "False"
```
