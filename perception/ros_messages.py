"""ROS message construction kept separate for offline contract tests."""
def messages(snapshot):
    import rospy
    from std_msgs.msg import Header
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs import point_cloud2
    header=Header();header.frame_id=snapshot['frame'];header.stamp=rospy.Time.from_sec(snapshot['source_stamp'])
    grid=OccupancyGrid();grid.header=header;grid.info.resolution=snapshot['resolution']
    grid.info.width=grid.info.height=snapshot['occupancy'].shape[0]
    grid.info.origin.position.x,grid.info.origin.position.y=snapshot['origin']
    grid.info.origin.orientation.w=1.;grid.data=snapshot['occupancy'].ravel().tolist()
    result=dict(map=grid,elevation=point_cloud2.create_cloud_xyz32(header,snapshot['elevation_points'].tolist()),
                obstacles=point_cloud2.create_cloud_xyz32(header,snapshot['obstacle_points'].tolist()))
    cost = OccupancyGrid()
    cost.header = header
    cost.info = grid.info
    cost.data = (snapshot['traversability'].ravel().tolist() if snapshot['valid'] else [-1]*len(grid.data))
    result['traversability'] = cost
    if snapshot['valid'] and snapshot.get('body_pose'):
        p=snapshot['body_pose'];pose=PoseStamped();pose.header.frame_id=snapshot['frame'];pose.header.stamp=rospy.Time.from_sec(p['stamp'])
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p['position']
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=p['quaternion']
        result['body_pose']=pose
    return result


def pointcloud_xyz(msg):
    """Decode XYZ as NumPy views, respecting ROS field offsets, endian and row padding."""
    import numpy as np
    if not hasattr(msg,'data'):
        # ROS-like test/custom transports can supply read_points without binary storage.
        from sensor_msgs import point_cloud2
        return np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=False)),float)
    if msg.width==0 or msg.height==0:return np.empty((0,3))
    fields={f.name:f for f in msg.fields};formats=[];offsets=[]
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or f.count!=1 or f.datatype not in (7,8):raise ValueError('unsupported XYZ field')
        size=4 if f.datatype==7 else 8
        if f.offset<0 or f.offset+size>msg.point_step:raise ValueError('invalid XYZ field offset')
        formats.append(('>' if msg.is_bigendian else '<')+('f4' if size==4 else 'f8'));offsets.append(f.offset)
    if msg.row_step<msg.width*msg.point_step or len(msg.data)<(msg.height-1)*msg.row_step+msg.width*msg.point_step:
        raise ValueError('truncated or invalid PointCloud2 row layout')
    dtype=np.dtype(dict(names=['x','y','z'],formats=formats,offsets=offsets,itemsize=msg.point_step))
    view=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    return np.column_stack([view[name].ravel() for name in ('x','y','z')]).astype(float,copy=False)


def terrain_overlay(snapshot, header):
    """Color measured ground tiles by the cost from the same map revision."""
    import numpy as np
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.header = header
    marker.ns = 'terrain_cost_at_elevation'
    marker.type = Marker.CUBE_LIST
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = snapshot['resolution'] * .96
    marker.scale.z = .018
    marker.color.a = 1.
    marker.action = Marker.DELETE
    if not snapshot['valid']:
        return marker
    ground = snapshot['elevation_points']
    history = snapshot.get('historical_elevation_points', ())
    if not len(ground) and not len(history):
        return marker
    cells = np.floor((ground[:, :2]-snapshot['origin'])/snapshot['resolution']).astype(int)
    costs = snapshot['traversability'][cells[:, 1], cells[:, 0]]
    marker.action = Marker.ADD
    for xyz in history:
        marker.points.append(Point(*xyz))
        marker.colors.append(ColorRGBA(.32, .36, .41, 1.))
    for xyz, cost in zip(ground, costs):
        marker.points.append(Point(*xyz))
        if cost < 0:
            color = (.4, .4, .4)
        elif cost == 100:
            color = (1., 0., 0.)
        else:
            # Keep the highest finite cost orange so it cannot look prohibited.
            risk = float(cost)/99.
            color = (min(1., 2*risk), 1.-.62*max(0., 2*risk-1.), .04)
        marker.colors.append(ColorRGBA(*color, 1.))
    return marker
