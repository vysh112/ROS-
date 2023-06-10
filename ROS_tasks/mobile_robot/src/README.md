PCL Overview:

The code subscribes to the /kinect/depth/points topic, which provides the input point cloud data from a Kinect sensor. The cloudCallback function is called whenever a new point cloud message is received.

The main steps of the algorithm are as follows:

    Convert the received sensor_msgs::PointCloud2 message to a pcl::PointCloud<pcl::PointXYZRGB> point cloud format.
    Preprocess the point cloud by filtering out points outside a specified range.
    Perform ground plane elimination using RANSAC (Random Sample Consensus) to separate the ground plane from the rest of the objects.
    Extract the objects (non-ground points) from the filtered cloud.
    Perform cluster extraction to group nearby points into separate clusters based on their spatial proximity.
    Color each segmented cluster object differently for visualization purposes.
    Convert the colored point cloud back to sensor_msgs::PointCloud2 format.
    Publish the segmented and colored point cloud on the output_cloud topic.

link to youtube video:

https://www.youtube.com/watch?v=3ADccpJb30E&ab_channel=VyshnavDavanagere

https://www.youtube.com/watch?v=gRmwRVPQdLI&ab_channel=VyshnavDavanagere
