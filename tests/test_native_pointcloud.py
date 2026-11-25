import struct
from types import SimpleNamespace as NS
import unittest
from native.pointcloud import compact_registered_cloud


def cloud(records, width=None, height=1, padding=b'', endian='<', double=False):
    step = 40 if double else 32
    width = len(records) if width is None else width
    rows = [b''.join(records[i*width:(i+1)*width])+padding for i in range(height)]
    return NS(width=width, height=height, point_step=step, row_step=width*step+len(padding),
              data=b''.join(rows), is_bigendian=endian=='>', is_dense=False,
              fields=[NS(name=n, offset=i*(8 if double else 4), datatype=8 if double else 7, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))])


def record(x, y, z, endian='<', double=False, extra=17):
    # The rest of the record represents intensity, ring, time and opaque padding.
    return struct.pack(endian+('ddd' if double else 'fff'), x, y, z)+bytes([extra])*(16 if double else 20)


class NativePointCloudTests(unittest.TestCase):
    def test_fixed_prefix_removed_and_every_valid_byte_preserved(self):
        valid = [record(i*.1+.1, -i*.01, .3, extra=i%255) for i in range(607)]
        m = cloud([record(0, 0, 0)]*10000+valid)
        original = m.data
        out, stats = compact_registered_cloud(m)
        self.assertEqual(out['data'], b''.join(valid))
        self.assertEqual(m.data, original)
        self.assertEqual((out['width'], out['height'], out['row_step']), (607, 1, 607*32))
        self.assertEqual(stats['removed_zero_points'], 10000)
        self.assertEqual(stats['removed_nonfinite_points'], 0)

    def test_scattered_zeros_nan_inf_and_signed_zero(self):
        keep = record(1, 2, 3)
        m = cloud([record(0, 0, 0), keep, record(float('nan'), 1, 2),
                   record(1, float('inf'), 2), record(-0., 0, 0)])
        out, stats = compact_registered_cloud(m)
        self.assertEqual(out['data'], keep)
        self.assertEqual(stats['removed_zero_points'], 2)
        self.assertEqual(stats['removed_nonfinite_points'], 2)

    def test_row_padding_is_not_interpreted_as_a_point(self):
        a, b = record(1, 0, 0), record(0, 1, 0)
        out, stats = compact_registered_cloud(cloud([a, record(0, 0, 0), b, record(0, 0, 0)],
                                                   width=2, height=2, padding=b'ROWPAD'))
        self.assertEqual(out['data'], a+b)
        self.assertEqual(stats['output_points'], 2)
        self.assertEqual(out['row_step'], 64)

    def test_big_endian_float64_with_remaining_record_bytes(self):
        a = record(1, -.2, 0, endian='>', double=True)
        out, _ = compact_registered_cloud(cloud([record(0, 0, 0, endian='>', double=True), a],
                                                endian='>', double=True))
        self.assertEqual(out['data'], a)
        self.assertEqual(out['point_step'], 40)

    def test_valid_organized_cloud_is_byte_identical(self):
        m = cloud([record(1, 2, 3)]*4, width=2, height=2, padding=b'pad')
        out, stats = compact_registered_cloud(m)
        self.assertEqual(out['data'], m.data)
        self.assertEqual((out['width'], out['height'], out['row_step']), (2, 2, 67))
        self.assertEqual(stats['removed_zero_points'], 0)

    def test_all_invalid_and_empty_are_valid_empty_layouts(self):
        for m in (cloud([record(0, 0, 0)]*5), cloud([])):
            out, _ = compact_registered_cloud(m)
            self.assertEqual((out['width'], out['height'], out['row_step'], out['data']), (0, 1, 0, b''))

    def test_bad_layout_or_missing_xyz_is_rejected(self):
        for field, value in [('row_step', 1), ('data', b'bad'), ('fields', []), ('width', -1)]:
            m = cloud([record(1, 2, 3)])
            setattr(m, field, value)
            with self.assertRaises(ValueError):
                compact_registered_cloud(m)

    def test_xyz_field_offsets_and_types_are_checked(self):
        for attr, value in [('offset', 33), ('datatype', 2), ('count', 2)]:
            m = cloud([record(1, 2, 3)])
            setattr(m.fields[0], attr, value)
            with self.assertRaises(ValueError):
                compact_registered_cloud(m)
        m = cloud([record(1, 2, 3)])
        m.fields[1].offset = 0
        with self.assertRaises(ValueError):
            compact_registered_cloud(m)


if __name__ == '__main__':
    unittest.main()
