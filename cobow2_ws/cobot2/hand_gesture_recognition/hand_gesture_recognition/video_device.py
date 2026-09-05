import os

import cv2


def resolve_camera_device():
    """사용할 카메라 장치 인덱스를 결정합니다.

    audio_device.py의 resolve_input_device()와 동일한 패턴입니다.
    - 환경변수 VISION_CAM_DEVICE 가 지정되어 있으면 그것을 최우선으로 사용합니다.
    - 없으면 0번부터 순서대로 열어보고, 정상적으로 열리는 첫 장치를 사용합니다.
    """
    override = os.environ.get("VISION_CAM_DEVICE")
    if override:
        return int(override) if override.isdigit() else override

    try:
        for idx in range(10):
            cap = cv2.VideoCapture(idx)
            if cap is not None and cap.isOpened():
                cap.release()
                return idx
    except Exception:
        pass
    return None
