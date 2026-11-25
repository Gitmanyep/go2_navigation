"""Compact registered native PointCloud2 records without altering valid bytes.

Only used for native cloud_deskewed. Exact XYZ zero vectors and nonfinite XYZ
are invalid observations here; all other fields and per-point padding survive.
"""
import numpy as np


def compact_registered_cloud(msg):
    width, height, step, row = msg.width, msg.height, msg.point_step, msg.row_step
    if any(type(v) is not int or v < 0 for v in (width, height, step, row)):
        raise ValueError('invalid_cloud_dimensions')
    count = width*height
    if count > 1000000 or len(msg.data) > 32000000:
        raise ValueError('oversized_cloud')
    blob = bytes(msg.data)
    stats = dict(input_points=count, input_bytes=len(blob), removed_zero_points=0,
                 removed_nonfinite_points=0, output_points=0, output_bytes=0)
    if count == 0:
        return dict(width=0, height=1, point_step=step, row_step=0, data=b'', is_dense=True), stats
    if step == 0 or row < width*step or len(blob) < (height-1)*row+width*step:
        raise ValueError('truncated_or_invalid_cloud_layout')
    fields = {f.name: f for f in msg.fields}
    if len(fields) != len(msg.fields):
        raise ValueError('duplicate_cloud_fields')
    offsets, formats = [], []
    for name in ('x', 'y', 'z'):
        f = fields.get(name)
        if f is None or f.count != 1 or f.datatype not in (7, 8):
            raise ValueError('unsupported_cloud_xyz_field')
        size = 4 if f.datatype == 7 else 8
        if f.offset < 0 or f.offset+size > step:
            raise ValueError('invalid_cloud_xyz_offset')
        if any(max(f.offset, lo) < min(f.offset+size, hi) for lo, hi in offsets):
            raise ValueError('overlapping_cloud_xyz_fields')
        offsets.append((f.offset, f.offset+size))
        formats.append(('>' if msg.is_bigendian else '<')+('f4' if size == 4 else 'f8'))
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats,
                          offsets=[a for a, _ in offsets], itemsize=step))
    view = np.ndarray((height, width), dtype=dtype, buffer=blob, strides=(row, step))
    finite = np.ones((height, width), bool)
    zero = np.ones((height, width), bool)
    for name in ('x', 'y', 'z'):
        finite &= np.isfinite(view[name])
        zero &= view[name] == 0.
    stats['removed_zero_points'] = int(zero.sum())
    stats['removed_nonfinite_points'] = int((~finite).sum())
    keep = finite & ~zero
    n = int(keep.sum())
    if n == count:
        # Preserve organized layout and row padding when no records are removed.
        out = dict(width=width, height=height, point_step=step, row_step=row,
                   data=blob, is_dense=True)
    else:
        records = np.ndarray((height, width), dtype='V'+str(step), buffer=blob,
                             strides=(row, step))
        data = records[keep].tobytes()
        out = dict(width=n, height=1, point_step=step, row_step=n*step,
                   data=data, is_dense=True)
    stats.update(output_points=n, output_bytes=len(out['data']))
    return out, stats
