#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
	cloud  = ros_to_pcl(pcl_msg)    

    # TODO: Statistical Outlier Filtering
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(50)
	x = 1.0
	outlier_filter.set_std_dev_mul_thresh(x)
	cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
        vox = cloud_filtered.make_voxel_grid_filter()
        LEAF_SIZE = 0.01
        vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
        cloud_filtered = vox.filter()

    # TODO: PassThrough Filter

        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = 0.6
        axis_max = 0.9
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'x'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = 0.35
        axis_max = 0.8
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'y'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = -0.45 
        axis_max = 0.45
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
        seg = cloud_filtered.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        max_distance = 0.007
        seg.set_distance_threshold(max_distance)



    # TODO: Extract inliers and outliers
        inliers, coefficients = seg.segment()
        extracted_inliers = cloud_filtered.extract(inliers, negative=False)
        extracted_outliers = cloud_filtered.extract(inliers, negative=True)
        cloud_table = extracted_inliers
        cloud_objects = extracted_outliers


    # TODO: Euclidean Clustering
        white_cloud = XYZRGB_to_XYZ(cloud_objects)
        tree = white_cloud.make_kdtree()


    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
        ec = white_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.01)
        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(50000)
        # Search the k-d tree for clusters
        ec.set_SearchMethod(tree)
        # Extract indices for each of the discovered clusters
        cluster_indices = ec.Extract()
        cluster_color = get_color_list(len(cluster_indices))

        color_cluster_point_list = []
        for j, indices in enumerate(cluster_indices):
                for i, indice in enumerate(indices):
                        color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

        #Create new cloud containing all clusters, each with unique color
        cluster_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
        ros_cloud_objects = pcl_to_ros(cloud_objects)
        ros_cloud_table = pcl_to_ros(cloud_table)
        ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
        pcl_objects_pub.publish(ros_cloud_objects)
        pcl_table_pub.publish(ros_cloud_table)
        pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
        detected_objects_labels = []
        detected_objects = []
        for index, pts_list in enumerate(cluster_indices):
                # Grab the points for the cluster from the extracted outliers (cloud_objects)
                pcl_cluster = cloud_objects.extract(pts_list)
                # TODO: convert the cluster from pcl to ROS using helper function
                ros_cluster = pcl_to_ros(pcl_cluster)


                # Extract histogram features
                # TODO: complete this step just as is covered in capture_features.py
                chists = compute_color_histograms(ros_cluster, using_hsv=True)
                normals = get_normals(ros_cluster)
                nhists = compute_normal_histograms(normals)
                feature = np.concatenate((chists, nhists))
                #print feature.shape
                #labeled_features.append([feature, model_name])


                # Make the prediction, retrieve the label for the result
                # and add it to detected_objects_labels list
                prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
                label = encoder.inverse_transform(prediction)[0]
                detected_objects_labels.append(label)
                #print label
                # Publish a label into RViz
                label_pos = list(white_cloud[pts_list[0]])
                label_pos[2] += .4
                #print label_pos
                object_markers_pub.publish(make_label(label,label_pos, index))

                # Add the detected object to the list of detected objects.
                do = DetectedObject()
                do.label = label
                do.cloud = ros_cluster
                detected_objects.append(do)

        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))


    # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    	try:
    		pr2_mover(detected_objects)
    	except rospy.ROSInterruptException:
        	pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
	from std_msgs.msg import Int32
	test_scene_num = Int32()
	from std_msgs.msg import String
	object_name = String()
	object_group = String()
	arm = String()
	pos_left = String()
	pos_right = String()
        # import message type
        from geometry_msgs.msg import Pose
        # initialize an empty pose message
        pick_pose = Pose()
        place_pose = Pose()

    # TODO: Get/Read parameters
    	object_list_param = rospy.get_param('/object_list')
    	dropbox_list_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables
        pos_left = dropbox_list_param[0]['position'] 
        pos_right = dropbox_list_param[1]['position']
        print pos_left
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
   	dict_list = []
        print len(object_list_param)
        print len(object_list)
        for i in range(0, len(object_list_param)):
	# Populate the data field
		object_name.data = object_list_param[i]['name']	
		object_group.data = object_list_param[i]['group']	
		#print object_name.data
        # TODO: Get the PointCloud for a given object and obtain it's centroid
	        labels = []
	        centroids = [] # to be list of tuples (x, y, z)
                found = False
		for object in object_list:
			if (object.label == object_name.data):
    				labels.append(object.label)
    				points_arr = ros_to_pcl(object.cloud).to_array()
    				centroids.append(np.mean(points_arr, axis=0)[:3])
                                x,y,z=centroids[0]
                                pick_pose.position.x=np.asscalar(x)
                                pick_pose.position.y=np.asscalar(y)
                                pick_pose.position.z=np.asscalar(z)
                                found = True
                                break
                if found: 
                # TODO: Create 'place_pose' for the object
                
                # TODO: Assign the arm to be used for pick_place
                    if(object_group.data == 'green'):
                        arm.data = 'right'
                        place_pose.position.x = pos_right[0]
                        place_pose.position.y = pos_right[1]
                        place_pose.position.z = pos_right[2]
                    else:
                        arm.data = 'left'
                        place_pose.position.x = pos_left[0]
                        place_pose.position.y = pos_left[1]
                        place_pose.position.z = pos_left[2]

                # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
		    test_scene_num.data = 3 
		    # Populate various ROS messages
    		    yaml_dict = make_yaml_dict(test_scene_num, arm, object_name, pick_pose, place_pose)
    		    dict_list.append(yaml_dict)
                
	#send_to_yaml("output1.yaml",dict_list)
	#send_to_yaml("output2.yaml",dict_list)
	send_to_yaml("output3.yaml",dict_list)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    # TODO: ROS node initialization
       	rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
        object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=10)
        detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=100)
        pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=10)
        pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=10)
        pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=10)

    # TODO: Load Model From disk
        #model = pickle.load(open('model1.sav', 'rb'))
        #model = pickle.load(open('model2.sav', 'rb'))
        model = pickle.load(open('model3.sav', 'rb'))
        clf = model['classifier']
        encoder = LabelEncoder()
        encoder.classes_ = model['classes']
        scaler = model['scaler']


    # Initialize color_list
	get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
        while not rospy.is_shutdown():
                rospy.spin()


