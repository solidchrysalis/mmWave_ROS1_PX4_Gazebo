import open3d as o3d 
import numpy as np
import bagreader as bag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def pointcloud_bag(filename):
    TOPIC = "/lidar_to_mmwave_pcl"
    tstart = None
    tend = None
    type_to_look ="sensor_msgs/PointCloud2"
    b = bag.bagreader(filename)
    print(b.topic_table)
    table_rows = b.topic_table[b.topic_table['Types']==type_to_look]
    topics_to_read = table_rows['Topics'].values
    message_counts = table_rows['Message Count'].values
    #print(topics_to_read)
    
    point_cloud_data = []
    for i in range(len(table_rows)):
        for topic, msg, t in b.reader.read_messages(topics=TOPIC, start_time=tstart, end_time=tend):
            points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            point_cloud_data.extend(points)
    # print(point_cloud_data)
    # Convert list to a NumPy array
    cloud_array = np.array(point_cloud_data, dtype=np.float32)

    # Convert to Open3D format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud_data)

    # Save as a .pcd file
    o3d.io.write_point_cloud("output.pcd", pcd)
    print("Saved point cloud to output.pcd")     

def main():
    pointcloud_bag("./test/2025-04-05-20-55-39.bag")

if __name__ == "__main__":
    main()

