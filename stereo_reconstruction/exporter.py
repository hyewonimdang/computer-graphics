"""Phase 6: 유한한 N×6 XYZRGB를 JSON/ASCII PLY/CSV로 저장."""
import csv
import json
from pathlib import Path
import os
import tempfile
import numpy as np


def _validated_cloud(points):
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 6 or not np.isfinite(cloud).all():
        raise ValueError('point cloud는 유한한 N×6 XYZRGB여야 합니다.')
    colors = cloud[:, 3:]
    if ((colors < 0) | (colors > 255) | (colors != np.floor(colors))).any():
        raise ValueError('RGB는 0..255 사이의 정수여야 합니다.')
    return cloud


def _atomic_write(path, writer):
    """같은 디렉터리의 임시 파일에 쓴 뒤 교체하여 불완전 파일을 줄인다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', newline='',
                                         dir=str(path.parent), delete=False) as handle:
            name = handle.name
            writer(handle)
        os.replace(name, str(path))
    finally:
        if name is not None and os.path.exists(name):
            os.unlink(name)
    return path


def export_json(points, path, units='unspecified'):
    """JSON object: schema_version, columns, count, coordinate_system, units, points.

    points 행=[X,Y,Z,R,G,B]. RGB는 JSON integer, XYZ는 number.
    allow_nan=False로 비표준 NaN/Infinity 출력을 금지한다. 빈 cloud도 지원.
    """
    cloud = _validated_cloud(points)
    if not isinstance(units, str):
        raise ValueError('units는 문자열이어야 합니다.')
    rows = [[float(x),float(y),float(z),int(r),int(g),int(b)] for x,y,z,r,g,b in cloud]
    data = dict(schema_version=1, columns=['X','Y','Z','R','G','B'], count=len(cloud),
                coordinate_system='left_camera: +X right, +Y down, +Z forward',
                units=units, color_encoding='RGB uint8 (0..255)', points=rows)
    return _atomic_write(path, lambda f: json.dump(data, f, ensure_ascii=False, allow_nan=False, indent=2))


def export_ply(points, path):
    """PLY ASCII 1.0: double XYZ + uchar RGB. 17자리로 float64 정밀도 유지."""
    cloud = _validated_cloud(points)
    def write(f):
        f.write('ply\nformat ascii 1.0\ncomment coordinates: left camera; units in companion JSON\n')
        f.write(f'element vertex {len(cloud)}\n')
        f.write('property double x\nproperty double y\nproperty double z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for x,y,z,r,g,b in cloud:
            f.write(f'{x:.17g} {y:.17g} {z:.17g} {int(r)} {int(g)} {int(b)}\n')
    return _atomic_write(path, write)


def export_csv(points, path):
    cloud = _validated_cloud(points)
    def write(f):
        writer = csv.writer(f)
        writer.writerow(['X','Y','Z','R','G','B'])
        writer.writerows([float(x),float(y),float(z),int(r),int(g),int(b)] for x,y,z,r,g,b in cloud)
    return _atomic_write(path, write)
