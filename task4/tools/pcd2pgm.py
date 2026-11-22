#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCD 点云转 ROS 2 栅格地图（.pgm + .yaml）v2

相比 v1 的改进：
    1. 自动估计地面高度（z 直方图峰值），不再手写 z_min
    2. 连通域过滤：孤立的一两格噪点直接删掉，墙体（大连通块）保留
    3. 封闭区域填充：被墙体围起来、但雷达没打到的内部区域标为可通行
       （Nav2 在 track_unknown_space=true 时不会规划进未知区，不清掉会走不动）

用法：
    python3 pcd2pgm.py scans.pcd map.pgm map.yaml
    python3 pcd2pgm.py scans.pcd map.pgm map.yaml --resolution 0.05 --min-cluster 6
"""
import argparse
import sys
from pathlib import Path

import numpy as np

_TYPE_MAP = {
    ('F', 4): 'f4', ('F', 8): 'f8',
    ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4', ('I', 8): 'i8',
    ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('U', 8): 'u8',
}

NEIGHBORS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def read_pcd(path):
    """解析 PCD，返回 Nx3 的 float32 坐标数组。"""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f'找不到文件：{path}')

    header = {}
    n_header_lines = 0
    with open(path, 'rb') as f:
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError('PCD 不完整：没有读到 DATA 行')
            n_header_lines += 1
            line = raw.decode('utf-8', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == 'DATA':
                break
        data_offset = f.tell()

    if 'FIELDS' not in header or 'DATA' not in header:
        raise ValueError('PCD 头部缺少 FIELDS 或 DATA')

    fields = header['FIELDS']
    sizes = [int(s) for s in header.get('SIZE', ['4'] * len(fields))[:len(fields)]]
    types = header.get('TYPE', ['F'] * len(fields))[:len(fields)]
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))[:len(fields)]]
    n_points = int(header.get('POINTS', ['0'])[0])
    data_kind = header['DATA'][0].lower()

    for need in ('x', 'y', 'z'):
        if need not in fields:
            raise ValueError(f'PCD 缺少字段 {need}，实际字段：{fields}')

    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        base = _TYPE_MAP.get((typ.upper(), size))
        if base is None:
            raise ValueError(f'不支持的字段类型：{name} TYPE={typ} SIZE={size}')
        dtype_fields.append((name, base, (count,)) if count > 1 else (name, base))
    dtype = np.dtype(dtype_fields)

    if data_kind == 'ascii':
        data = np.loadtxt(path, dtype=dtype, skiprows=n_header_lines - 1, ndmin=1)
    elif data_kind == 'binary':
        raw = np.fromfile(path, dtype=np.uint8, offset=data_offset)
        need_bytes = dtype.itemsize * n_points
        if raw.size < need_bytes:
            raise ValueError(
                f'二进制数据不足：需要 {need_bytes} 字节，实际只有 {raw.size} 字节'
            )
        data = np.frombuffer(raw[:need_bytes].tobytes(), dtype=dtype, count=n_points)
    else:
        raise ValueError(f'暂不支持 DATA={data_kind}（支持 ascii / binary）')

    xyz = np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)
    return xyz.reshape(-1, 3)


def estimate_floor(xyz):
    """用 z 方向直方图的峰值估计地面高度。"""
    z = xyz[:, 2]
    lo, hi = np.percentile(z, [1, 60])
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 0.05:
        return float(np.median(z))
    bins = np.arange(lo, hi + 0.02, 0.02)
    hist, edges = np.histogram(z, bins=bins)
    peak = int(np.argmax(hist))
    return float((edges[peak] + edges[peak + 1]) / 2)


def filter_clusters(mask, min_size):
    """去掉面积小于 min_size 的连通块。"""
    if min_size <= 1:
        return mask
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    keep = np.zeros_like(mask, dtype=bool)
    for start_y in range(height):
        for start_x in range(width):
            if not mask[start_y, start_x] or visited[start_y, start_x]:
                continue
            stack = [(start_y, start_x)]
            visited[start_y, start_x] = True
            comp = []
            while stack:
                y, x = stack.pop()
                comp.append((y, x))
                for dy, dx in NEIGHBORS:
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width \
                            and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            if len(comp) >= min_size:
                for y, x in comp:
                    keep[y, x] = True
    return keep


def dilate(mask, radius):
    """方形膨胀。"""
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    grown = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            shifted = np.zeros_like(mask)
            ys = slice(max(dy, 0), height + min(dy, 0))
            yd = slice(max(-dy, 0), height + min(-dy, 0))
            xs = slice(max(dx, 0), width + min(dx, 0))
            xd = slice(max(-dx, 0), width + min(-dx, 0))
            shifted[yd, xd] = mask[ys, xs]
            grown |= shifted
    return grown


def fill_bbox(occupied, inset=1):
    """把障碍物外接矩形内部、没被占据的格子标为可通行。

    比赛场地四周是连续的围挡，外接矩形正好套住场地，
    框内空着的部分就是能走的地面，比"漫水判封闭"稳得多。
    """
    ys, xs = np.where(occupied)
    if ys.size == 0:
        return np.zeros_like(occupied, dtype=bool)
    y0, y1 = ys.min() + inset, ys.max() - inset
    x0, x1 = xs.min() + inset, xs.max() - inset
    if y1 <= y0 or x1 <= x0:
        return np.zeros_like(occupied, dtype=bool)
    box = np.zeros_like(occupied, dtype=bool)
    box[y0:y1 + 1, x0:x1 + 1] = True
    return box & ~occupied


def build_grid(xyz, resolution, z_offset, z_max_rel, min_cluster, inflate):
    pts = xyz[np.isfinite(xyz).all(axis=1)]
    if pts.size == 0:
        raise ValueError('点云为空或全是 NaN')

    floor = estimate_floor(pts)
    z_min = floor + z_offset
    z_max = floor + z_max_rel

    margin = max(resolution * 2, 0.2)
    x_min = float(pts[:, 0].min()) - margin
    y_min = float(pts[:, 1].min()) - margin
    x_max = float(pts[:, 0].max()) + margin
    y_max = float(pts[:, 1].max()) + margin
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    band = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]
    if band.size == 0:
        raise ValueError(
            f'地面估计 {floor:.2f} m，高度区间 [{z_min:.2f}, {z_max:.2f}] 内没有点；'
            f'实际高度范围 {pts[:, 2].min():.2f} ~ {pts[:, 2].max():.2f} m'
        )

    col = np.floor((band[:, 0] - x_min) / resolution).astype(np.int64)
    row = height - 1 - np.floor((band[:, 1] - y_min) / resolution).astype(np.int64)
    ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    occupied = np.zeros((height, width), dtype=bool)
    occupied[row[ok], col[ok]] = True

    raw_cells = int(occupied.sum())
    occupied = filter_clusters(occupied, min_cluster)
    after_filter = int(occupied.sum())
    if inflate > 0:
        occupied = dilate(occupied, inflate)

    # 判断"封闭"时把墙加厚一点，防止墙角或个别缺格漏成开口
    free = fill_bbox(occupied, inset=1)

    image = np.full((height, width), 205, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0

    info = {
        'floor': floor,
        'z_min': z_min,
        'z_max': z_max,
        'raw_cells': raw_cells,
        'after_filter': after_filter,
    }
    return image, x_min, y_min, info


def write_outputs(image, x_min, y_min, resolution, pgm_path, yaml_path):
    height, width = image.shape
    pgm_path = Path(pgm_path)
    yaml_path = Path(yaml_path)
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)

    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# generated by pcd2pgm.py\n')
        f.write(f'{width} {height}\n'.encode())
        f.write(b'255\n')
        f.write(image.tobytes())

    yaml_text = (
        f'image: {pgm_path.name}\n'
        f'resolution: {resolution}\n'
        f'origin: [{x_min:.6f}, {y_min:.6f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n'
    )
    yaml_path.write_text(yaml_text, encoding='utf-8')
    return width, height


def main():
    parser = argparse.ArgumentParser(description='PCD → ROS 2 栅格地图（v2）')
    parser.add_argument('pcd', help='输入 .pcd 文件')
    parser.add_argument('pgm', help='输出 .pgm 文件')
    parser.add_argument('yaml', help='输出 .yaml 文件')
    parser.add_argument('--resolution', type=float, default=0.05, help='分辨率 米/格，默认 0.05')
    parser.add_argument('--z-offset', type=float, default=0.15,
                        help='障碍物判定相对地面的最低高度，默认 0.15 m')
    parser.add_argument('--z-max', type=float, default=2.0,
                        help='障碍物判定相对地面的最高高度，默认 2.0 m')
    parser.add_argument('--min-cluster', type=int, default=4,
                        help='连通块最小格数，小于此值视为噪点，默认 4')
    parser.add_argument('--inflate', type=int, default=1, help='障碍物膨胀格数，默认 1')
    args = parser.parse_args()

    if args.resolution <= 0:
        sys.exit('分辨率必须大于 0')

    xyz = read_pcd(args.pcd)
    image, x_min, y_min, info = build_grid(
        xyz, args.resolution, args.z_offset, args.z_max, args.min_cluster, args.inflate
    )
    width, height = write_outputs(image, x_min, y_min, args.resolution, args.pgm, args.yaml)

    occ = int((image == 0).sum())
    free = int((image == 254).sum())
    unknown = int((image == 205).sum())
    print(f'输入点数        : {len(xyz)}')
    print(f'估计地面高度    : {info["floor"]:.2f} m（障碍物取 {info["z_min"]:.2f} ~ {info["z_max"]:.2f} m）')
    print(f'障碍格          : 原始 {info["raw_cells"]} → 过滤噪点后 {info["after_filter"]}')
    print(f'栅格尺寸        : {width} × {height}（{width * args.resolution:.1f} m × '
          f'{height * args.resolution:.1f} m）')
    print(f'占用 / 空闲 / 未知: {occ} / {free} / {unknown}')
    print(f'origin          : [{x_min:.3f}, {y_min:.3f}, 0.0]')
    print(f'已写入          : {args.pgm}  与  {args.yaml}')


if __name__ == '__main__':
    main()
