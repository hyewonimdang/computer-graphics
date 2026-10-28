"""여러 깊이의 작은 정면 텍스처 패치로 재현 가능한 stereo 입력 생성.

정렬된 카메라의 disparity=f*baseline/Z를 흉내 낸다. 실제 촬영 이미지가
아니며 이미지 파이프라인의 smoke test 용도다. 수학 검증은 tests에서
알려진 3D 좌표를 두 카메라에 투영한 대응점으로 별도 수행한다.
"""
import argparse
from pathlib import Path
import numpy as np
from image_io import save_image


def generate_demo(output):
    rng = np.random.default_rng(123)
    left = np.full((600, 900, 3), 25, np.uint8)
    right = left.copy()
    for y in range(60, 520, 90):
        for x in range(80, 820, 100):
            patch = rng.integers(0, 256, (54, 54, 3), dtype=np.uint8)
            disparity = int(rng.integers(8, 34))
            left[y:y + 54, x:x + 54] = patch
            right[y:y + 54, x - disparity:x - disparity + 54] = patch
    save_image(Path(output) / 'left.png', left)
    save_image(Path(output) / 'right.png', right)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='demo')
    generate_demo(parser.parse_args().output)
