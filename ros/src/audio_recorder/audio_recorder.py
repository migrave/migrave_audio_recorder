import wave
from pathlib import Path
import datetime
import rospy

class AudioRecorder:
    def __init__(
        self,
        audio_rate=16000,
        audio_channels=1,
        audio_width=2,
        audio_type="wav",
        out_directory="audio",
    ):
        self._audio_rate = audio_rate
        self._audio_channels = audio_channels
        self._audio_width = audio_width
        self._audio_type = audio_type

        self._out_directory = out_directory

        self._is_recording = False
        self._timestamp_writer = None

    def __del__(self):
        if self._timestamp_writer is not None:
            self._timestamp_writer.close()

    def start_recording(self, out_file_name=None):

        if self._is_recording:
            raise RuntimeError("Audio is already being recorded")

        # parameter set by migrave_games
        parameter_name = "/migrave/game_performance/participant_id"
        if rospy.has_param(parameter_name):
            participant_id = rospy.get_param(parameter_name)
            if participant_id == "":
                rospy.loginfo("Participant ID not set!")
                participant_id = "ID_Unknown"
        else:
            participant_id = "ID_Unknown"

        self._out_directory_id = Path(self._out_directory) / participant_id
        if not Path(self._out_directory_id).is_dir():
            Path(self._out_directory_id).mkdir(parents=True, exist_ok=True)

        if out_file_name is None:
            ext = self._audio_type
            now = datetime.datetime.now()
            # time in YYYY-MM-DD_HH_MM-SS format
            time = now.strftime("%Y-%m-%d_%H-%M-%S")
            # unixtimestamp 16 digits
            stamp = int(datetime.datetime.timestamp(now) * 1000000)
            file_name = f"{time}_{stamp}"
            out_file_name = f"{file_name}.{ext}"
            timestamp_file_name = f"{file_name}.txt"

        out_file_path = Path(self._out_directory_id) / out_file_name
        out_file_path = str(out_file_path)
        self._wf = wave.open(out_file_path, "wb")
        self._wf.setnchannels(self._audio_channels)
        self._wf.setsampwidth(self._audio_width)
        self._wf.setframerate(self._audio_rate)

        timestamp_file_name = out_file_path[:-3] + 'txt'
        self._timestamp_writer = open(timestamp_file_name, "a")

        self._is_recording = True

    def add_audio(self, audio, is_throw_error_if_not_recording=True):
        if self._is_recording:

            now = datetime.datetime.now()
            # unixtimestamp 16 digits
            timestamp = int(datetime.datetime.timestamp(now) * 1000000)
            timestamp = str(timestamp) + "\n"
            self._timestamp_writer.write(timestamp)

            self._wf.writeframes(audio)
        else:
            if is_throw_error_if_not_recording:
                raise RuntimeError("Audio recording has not been started")

    def stop_recording(self):
        if not self._is_recording:
            raise RuntimeError("Audio recording was not started")

        self._wf.close()

        if self._timestamp_writer is not None:
            self._timestamp_writer.close()

        self._is_recording = False
