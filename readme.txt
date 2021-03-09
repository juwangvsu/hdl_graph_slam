3/8/21
	now focus on scanmatch having poor result.

	the time lag between ros and ue engine is a non issue. depthimg_pc2 
	should use the current ros time (or the ros time when depth image 
	callback is called).
	see airsim node readme

	2nd issue: even with gt odom, the map still not perfect due to 
	map->odom tf has errors. the 
	odom->base_link is from scanmatching
	base_link->front_left_custom_body/static is fixed.
		lidar2base_publisher
	the cloud(keypoints) is in the front_left_custom_body/static frame.
	so map->odom->base_link error messup the map 

	the map is published at
		it is a simple combine of all pc2 in all nodes(keyframes)
		/hdl_graph_slam/map_pointsa
		type: sensor_msgs/PointCloud2
		frame id "map"

3/6/21 
	building99 depthimg_pc2 works quite well. the scan match is good.

	build99_x10_dt.bag
		move xyz (-10,0,-2), time lag compensated.
	build99_rotate_dt.bag
		rotate, time lag compensated.	
	build99.bag
		at 21.6 second, 8-th node added to graph, that is a bad node,
		drone come to a stop and have pitch up/down, the pc data is not
		quat: 0,0.039,0,0.9992 adjusted accordingly, so the
corresponding node cause 
		he map degrade. 
		two methods to test: (1) dont add bad node at pitch roll motion
			(2) adjust pc data with drone orientation data pitch/roll 
	build99_rotate.bag
		two rotations: 8.5 sec, 16.5 sec
		depthimg_pc2 about 2hz,
		depthimg  slight lag behind depthimg_pc2
		
3/5/21  
	verify scan_match produced odom accuracy (relative). 
		so problem is scan_match, test with a simple environment.
	use bag data and recorded ground truth
		record bag for block game. rviz data examine: 
		visually examine the point cloud and its movement in 
		rviz seems indicating the cloud data is accurate
		so problem is scan_match produce a poor visual odom .
		some keyframe and odom should not be used to produce match, 
			esp when pitch/roll > certain degree.
			tbd 	
	scan_match nodelet publish airsim_odom as markers to provide a reference
		to check/compare with visual odom, done
	scan_match nodelet publish visual odom as markers to study
		tbd 
	new launch file only start filter and scan match nodelet and a tf so it will should side by side
	with gt odom
	/airsim_node/SimpleFlight/odom_local_ned

block	bag file: 25 sec- 37 sec, first 10 meter movement
		  55 sec - 68 sec, second 10 meter movement
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

		rosbag play --clock build99.bag --topics /airsim_node/SimpleFlight/depthimg_pc2 /airsim_node/SimpleFlight/odom_local_ned /tf /tf_static /airsim_node/SimpleFlight/depthimg_pc2:=/airsim_node/SimpleFlight/lidar/LidarCustom

		block_10m_5m.bag
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
prefiltering_nodelet:
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
	tf: sendTransform
		odom:   odom_frame_id=odom
			base_frame_id=cloud_msg->header.frame_id
			"odom->front_left_custom_body/static"
		keyframe:
			"odom->keyframe"
			don't know who use it.


	
2021-02-25-17-55-42.bag
2021-02-26-22-56-54.bag
-------------------------------------------------------------
hdl_graph_slam_nodelet:
	sub:
		/filtered_points --- sync with odom trigger cloud_callback()
		/odom
		/gpsimu_driver/imu_data
		/floor_detection/floor_coeffs
	pub:
		/hdl_graph_slam/markers
		/hdl_graph_slam/odom2pub
			this is converted to /tf:
				"map->odom"
				by python node map2odom_publisher.py
		/hdl_graph_slam/map_points
				frame_id: map
	srv:
		/hdl_graph_slam/dump
		/hdl_graph_slam/save_map

	two timers: 	optimize_timer_callback
				check new keyframe, put into keyframes[]
					keyframe.node, .cloud, .odom
				return if no keyframe update
			map_...timer_callback
				return if no subscriber or no graph change

--------------------------------------------------------
map2odom_publisher.py
	sub:
		/hdl_graph_slam/odom2pub
	tf:
		"map->odom"
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

--------------------------------------------
tf when hdl nodes runs:
two tf tree exist:
one tf published by airsim node, another by hdl nodelets.
tf from airsim:
	SimpleFlight-> ....
tf from hdl nodelets:
	map -> odom -> ...
conflict one:
	both tf on child frame front_left_custom_body/static

	map->odom (map2odom_publisher.py)
		 -> baselink (scan match nodelet)
			    -> front_left_custom_body/static (lidar2base static)
		 -> keyframe (scan match nodelet)

depthimg_pc2 visible on both SimpleFlight or map 
