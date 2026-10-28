"""이미지를 OpenCV BGR uint8 형식으로 읽고 저장한다."""
from pathlib import Path
import cv2
import numpy as np


def load_image(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"이미지 파일이 없습니다: {path}")
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"이미지를 해석할 수 없습니다: {path}")
    return image


def load_stereo_images(left_path, right_path):
    return load_image(left_path), load_image(right_path)


def save_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode(path.suffix, image)
    if not ok:
        raise OSError(f"이미지 인코딩 실패: {path}")
    data.tofile(str(path))
