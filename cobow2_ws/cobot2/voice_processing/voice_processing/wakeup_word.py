import os
import numpy as np
import sounddevice as sd
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory

from voice_processing.audio_device import resolve_input_device

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(PACKAGE_PATH, f"resource/{MODEL_NAME}")

SAMPLE_RATE = 16000
FRAME = 1280


class WakeupWord:
    def __init__(self):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None

    def is_wakeup(self):
        audio_chunk, _ = self.stream.read(FRAME)
        audio_chunk = audio_chunk.flatten()
        confidence = self.model.predict(audio_chunk)[self.model_name]
        print("confidence: ", confidence)
        if confidence > 0.3:
            print("Wakeword detected!")
            return True
        return False

    def open(self):
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype="int16", blocksize=FRAME,
            device=resolve_input_device(),
        )
        self.stream.start()

    def close(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None
