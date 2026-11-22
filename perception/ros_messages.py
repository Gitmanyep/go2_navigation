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
    if snapshot['valid'] and snapshot.get('body_pose'):
        p=snapshot['body_pose'];pose=PoseStamped();pose.header.frame_id=snapshot['frame'];pose.header.stamp=rospy.Time.from_sec(p['stamp'])
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p['position']
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=p['quaternion']
        result['body_pose']=pose
    return result
