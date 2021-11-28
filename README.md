# migrave_audio_recorder
Repository for a audio recorder used in the MigrAVE project

## Depedencies
Rely on `qt_respeaker_app_node` node (`/qt_respeaker_app/channel0` topic).

## Usage 

Start

```sh
rostopic pub /qt_robot_audio_recording/is_record std_msgs/Bool "True"
```
Stop

```sh
rostopic pub /qt_robot_audio_recording/is_record std_msgs/Bool "False"
```
