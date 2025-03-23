import pypcd4
import numpy as np
import bagpy
from sensor_msgs.msg import PointCloud2, PointField

def pointcloud_bag(filename):
    tstart = None
    tend = None
    type_to_look ="laser/scan"
    b = bagreader(filename)
    print(b)
    """
    table_rows = b.topic_table[b.topic_table['Types']==type_to_look]
    topics_to_read = table_rows['Topics'].values
    message_counts = table_rows['Message Count'].values
    print(message_counts)
    
    column_names = [
                            "headser.seq", 
                            "header.frame_id", 
                            "height"
                            "width"
                            "fields.name"
                            "fields.offset"
                            "fields.datatype"
                            "fields.count"
                            "is_bigendian"
                            "point_step"
                            "row_step"
                            "data"
                            "is_dense"
                            ]

    array = []
    for i in range(len(table_rows)):
        for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
            new_row = [t.secs + t.nsecs*1e-9, 
                                    msg.header.seq, 
                                    msg.header.frame_id,
                                    msg.child_frame_id,
                                    msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z,
                                    msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w,
                                    msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y,
                                    msg.twist.twist.linear.z]

    array.append(file_to_write)
    return csvlist
    """

def main():
    pointcloud_bag("video_log_2025-03-05-18-38-28.bag")

if __name__ == "__main__":
    main()





if __name__ == "__main__": 
	main()