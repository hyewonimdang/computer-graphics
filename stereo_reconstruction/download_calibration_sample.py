"""OpenCV 4.11.0 공식 stereo 체커보드 샘플 다운로드 (명시적으로 실행할 때만)."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

BASE = 'https://raw.githubusercontent.com/opencv/opencv/4.11.0/'
IDS = ['01','02','03','04','05','06','07','08','09','11','12','13','14']


def download(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    records = []
    names = [('samples/data/' + side + i + '.jpg', side + i + '.jpg')
             for i in IDS for side in ('left','right')]
    names += [('samples/data/stereo_calib.xml','stereo_calib.xml'), ('LICENSE','OPENCV_LICENSE.txt')]
    for source, local in names:
        url = BASE + source
        with urlopen(url, timeout=45) as response:
            data = response.read()
        target = output/local
        if target.exists() and target.read_bytes() != data:
            raise ValueError(f'기존 파일이 원본과 다릅니다. 새 폴더를 사용하세요: {target}')
        target.write_bytes(data)
        records.append(dict(file=local, source=url, sha256=hashlib.sha256(data).hexdigest()))
    def pairs(ids):
        return [dict(left='left'+i+'.jpg', right='right'+i+'.jpg') for i in ids]
    for name, ids in [('train_pairs.json',IDS[:-2]), ('validation_pairs.json',IDS[-2:])]:
        (output/name).write_text(json.dumps(pairs(ids),indent=2),encoding='utf-8')
    provenance = dict(repository='https://github.com/opencv/opencv', version='4.11.0',
                      board_inner_corners=[9,6], square_size=1.0, units='board_square',
                      scale_note='Physical square size not established; do not interpret as meters.',
                      training_pairs=11, held_out_pairs=2, files=records)
    (output/'provenance.json').write_text(json.dumps(provenance,indent=2),encoding='utf-8')
    print(f'Downloaded 13 pairs: {output}; train=11, held-out=2; units=board_square')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='sample_calibration')
    args=parser.parse_args()
    try:
        download(args.output)
    except (OSError,ValueError) as error:
        parser.exit(2,f'오류: {error}\n')
