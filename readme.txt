
----------------------------------------------------------------------
2/27/21
	scan_match nodelet , hdl slam nodelet result with airsim data
	2021-02-25-17-55-42.bag	--- depthimg_pc2 
	2021-02-26-22-56-54.bag --- depthimg_pc2 floor removed
	
	the scan match map is 70% good.
	the odom graph is also 70% good. result similar for both bag files
	the map is completely updated by the scan match nodelet. 
	the odom graph is depend on the map

	TBD: further deep dive to hdl nodelet and scan match nodelet.
	try to use imu_acc?
	modify rviz to plot ground truth odom from airsim data
----------------------------------------------------------------------
2/26/21
	hdl node crash with airsim bag file

	in launch file change an option from FAST_GICP to NDT seems cure.
	the point cloud seem is processed by FAST_GICP or NDT before the hdl nodelet code. to verify using isValid() func to examin point cloud. set to NDT so we have a change to see the data in nodelet.
      fast_gicp crash, invalid point, nan or inf, may due to bad transformatio
n trying to register same points?

	ros debugging a node or nodelet use launch_prefix="gdb..."

	--- status:
		one bug identified: in airsim_node, the depthimg_pc2 contain clipped points
		at max_range. these points should be removed. otherwise the 
		scan matching code here will be dominated by these points and
		generate a near (0,0,0) odom estimation.
		NDT algo produce something, FAST_GICP result bad, and still crash.
		the floor points might need to be removed too?
			floor points have little impact on scan match result.
   	Test steps:
		rosbag play --clock 2021-02-25-17-55-42.bag --topics /airsim_node/SimpleFlight/depthimg_pc2 /airsim_node/SimpleFlight/depthimg_pc2:=/airsim_node/SimpleFlight/lidar/LidarCustom
		roslaunch hdl_graph_slam hdl_graph_slam_airsim_lidar.launch 
		rviz -d hdl_graph_slam_airsim.rviz

----------------------------------------------------------------------
2/25/21
	rosbag play --clock 2021-02-24-18-52-53.bag --topics /airsim_node/SimpleFlight/depthimg_pc2
	roslaunch hdl_graph_slam hdl_graph_slam_airsim.launch 
	rviz can see map
/use_sim_time true
	this decide where the ros time or clock come from. wall clock vs
	sim clock
	this is necessary for some map2odom tf node to produce tf when rosbag.
	if false, the tf will not work in that node. the tf tree is broken

	when use airsim_node, this must be false, otherwise the node is not doing
	anything.


----------------------------------------------------------------------
2/24/21 hdl_graph_slam
	subscribe to /airsim_node/SimpleFlight/depthimg_pc2
		     /airsim_node/SimpleFlight/odom_local_ned

Test steps:
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
rosbag play --clock hdl_400.bag

-----------------------------------------
bag file topics:

~/gpsimu_driver/gpstime
/gpsimu_driver/imu_data
  frame_id: "/velodyne:"

/gpsimu_driver/nmea_sentence
/gpsimu_driver/temperature
/rosout
/rosout_agg
/velodyne_points
	  frame_id: "velodyne"
`
rostopic echo -n 1 /velodyne_points > velodyne_points.txt
rostopic echo -n 1 /gpsimu_driver/imu_data > imu_data.txt
------------------------------------------------------------
	sub: /velodyne_points, remapped in launch file
		tf: base_link to frame_id of point cloud /velodyne_points
			provided static in launch file
	pub: /filtered_points, frame id: "base_link" 
				launch file param
	
-------------------------------------------------------------
scan_matching_odometry_nodelet:
	sub: launch file says points_topic, which is used for offline.
		/filtered_points, pubed by prefilterring_nodelet
	pub: /odom
		/scan_matching_odometry/status
		/aligned_points

	
2021-02-25-17-55-42.bag
2021-02-26-22-56-54.bag
-------------------------------------------------------------
hdl_graph_slam_nodelet:
	sub:
	pub:

---------------------------------------------------------------
floor filter nodelet:
	sub: launch file says points_topic, which is used for offline.
		/filtered_points, pubed by prefilterring_nodelet
	pub:	/floor_detection/floor_points
		/floor_detection/floor_filtered_points
floor_filtered_points.txt
	points used to detect floor, after clipping
	frm floor_detection nodelet, 
	frame_id: base_link
floor_points.txt:
	floor points inlier (blue)
	frm floor_detection nodelet, 
	frame_id: base_link
dumped point clouds:
	~/.ros/filtered_points.pcd, floor_clip1.pcd floor_clip2.pcd

	alg: the filtered_points are in base_link frame, which is the same
		as frame_id velodyne here (launch file tf). so the point 
		can still have negative z value. 

		the dumped velodyne pc file shows: z value -4 to +12 meters, 
		floor is in negative range,
		above ground structure is positive. this is consistant with the 
		lidar sensor height at about 1.7 meters above ground.

		this does not seems agree with
		the way clip is done in code. to debug point's z value.
		400 bag file velodyne points assume normal body frame, so floor
		at -1 meters, and obstacles in postive range.

		the airsim depthimg_pc2 point cloud is assume body frame ned
		so floor is at around +1 meters, and obstacles in negative
		range.
		the result for airsim depthimg_pc2 is that the actual floor 
		points is removed. so maybe the the launch file need to 
		fix tf from ned to normal frame instead of a dummy 0/0/0 tf.
----------------------------------------------------------------	
 rostopic info /gpsimu_driver/imu_data
Type: sensor_msgs/Imu

Publishers: 
 * /play_1614200103623237315 (http://asus1:42507/)

Subscribers: 
 * /velodyne_nodelet_manager (http://asus1:45353/)

$ rostopic info /velodyne_points 
Type: sensor_msgs/PointCloud2

Publishers: 
 * /play_1614200103623237315 (http://asus1:42507/)

Subscribers: 
 * /velodyne_nodelet_manager (http://asus1:45353/)

/scan_matching_odometry_nodelet

rosnode info /velodyne_nodelet_manager 
--------------------------------------------------------------------------------
Node [/velodyne_nodelet_manager]
Publications: 
 * /aligned_points [sensor_msgs/PointCloud2]
 * /colored_points [sensor_msgs/PointCloud2]
 * /filtered_points [sensor_msgs/PointCloud2]
 * /hdl_graph_slam/map_points [sensor_msgs/PointCloud2]
 * /hdl_graph_slam/markers [visualization_msgs/MarkerArray]
 * /hdl_graph_slam/odom2pub [geometry_msgs/TransformStamped]
 * /hdl_graph_slam/read_until [std_msgs/Header]
 * /odom [nav_msgs/Odometry]
 * /rosout [rosgraph_msgs/Log]
 * /scan_matching_odometry/read_until [std_msgs/Header]
 * /scan_matching_odometry/transform [geometry_msgs/TransformStamped]
 * /tf [tf2_msgs/TFMessage]
 * /velodyne_nodelet_manager/bond [bond/Status]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /filtered_points [sensor_msgs/PointCloud2]
 * /floor_detection/floor_coeffs [unknown type]
 * /gpsimu_driver/imu_data [sensor_msgs/Imu]
 * /odom [nav_msgs/Odometry]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [unknown type]
 * /velodyne_nodelet_manager/bond [bond/Status]
 * /velodyne_points [sensor_msgs/PointCloud2]


