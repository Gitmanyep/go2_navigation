"""LidarState schema matching the installed controller-facing C++ SDK.

The original Python SDK snapshot lacks dirty_percentage. Keep that snapshot
unchanged and define the observed 18-field wire schema locally instead.
Reference: /home/unitree/unitree_sdk2/include/unitree/idl/go2/LidarState_.hpp.
Confirmed by independent C++ subscription and native DDS samples, 2026-11-25.
"""
from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid('sequential')
class LidarState_(idl.IdlStruct, typename='unitree_go.msg.dds_.LidarState_'):
    stamp: types.float64
    firmware_version: str
    software_version: str
    sdk_version: str
    sys_rotation_speed: types.float32
    com_rotation_speed: types.float32
    error_state: types.uint8
    dirty_percentage: types.uint8
    cloud_frequency: types.float32
    cloud_packet_loss_rate: types.float32
    cloud_size: types.uint32
    cloud_scan_num: types.uint32
    imu_frequency: types.float32
    imu_packet_loss_rate: types.float32
    imu_rpy: types.array[types.float32, 3]
    serial_recv_stamp: types.float64
    serial_buffer_size: types.uint32
    serial_buffer_read: types.uint32
