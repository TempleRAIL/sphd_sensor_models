#!/usr/bin/env python
import rospy
import tf
from scipy.optimize import linprog
import numpy as np
import rosbag
# custom define messages:
from sphd_msgs.msg import SPHDMeasurement, SPHDMeasurements

# target ground truth positions:
TARGETS_CLASS = ["Aquafina", "Nestle", "Aquafina", "Deer", 
                 "Kirkland", "Nestle", "Kirkland", "Nestle",
                 "Deer", "Deer", "Kirkland", "Aquafina"]
TARGETS = [[0.8664,-2.2737], [0.8586,-3.0388], [-2.1199,-2.1170], [-1.8117,-3.2306], 
           [-1.7558,-4.2157], [1.9538,-4.3119], [3.1313,-5.3481], [8.2407,-4.6337],
           [8.6104,-3.1557], [5.0591,-4.4601], [4.6538,-2.8504], [3.6701,1.3240]]
# noise parameters: 
SIGMA_RANGE = 3*0.25   # 3*r, r: 0.35
SIGMA_BEARING = 3*(np.pi/180)*3 # 3*b*pi/180, b: 3 degrees

# read rosbag and process the measurements:
def clutter_process(sigma_r, sigma_b):
    # clutter model information:
    # bearing:
    b_max = np.pi/4
    b_min = -np.pi/4
    BIN_NUM_B = 8
    BIN_SIZE_B = (b_max - b_min)/BIN_NUM_B
    # range:
    r_max = 1.5
    r_min = 0.6
    BIN_NUM_R = 6
    BIN_SIZE_R = (r_max - r_min)/BIN_NUM_R
    # true detection number in the bearing bin: -pi/4 ~ pi/4 / (pi/30) = 15, 0.4 ~ 3.1 / 0.3 = 9 
    bin_true = np.zeros([BIN_NUM_B, BIN_NUM_R])
    # false positive number in the bearing bin: -pi/4 ~ pi/4 / (pi/30) = 15
    bin_fp = np.zeros([BIN_NUM_B, BIN_NUM_R])
    # valid measurement number:
    meas_cnt = 0
    # clutter number per measurement:
    clutter_cnt = np.zeros(2725) 

    # target number:
    target_num = len(TARGETS)
    # open the rosbag:
    bag = rosbag.Bag('../../../measurement_model_dataset/bottle_measurement_model.bag')
    print("Finish reading the rosbag.")
    for topic, sphd_det_msg, t in bag.read_messages(topics=['/sphd_measurements']):
        # get robot pose:
        robot_trans = sphd_det_msg.q_pose.position
        robot_rot = [sphd_det_msg.q_pose.orientation.x, sphd_det_msg.q_pose.orientation.y,
                    sphd_det_msg.q_pose.orientation.z, sphd_det_msg.q_pose.orientation.w]
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(robot_rot)
        # homogeneous transformation matrix:
        map_T_robot = np.array([[np.cos(theta), -np.sin(theta), robot_trans.x],
                                    [np.sin(theta), np.cos(theta),robot_trans.y],
                                    [0, 0, 1]])
        # valid measurement set flag:
        flag_valid = 0
        cl_cnt = 0
        # association flag:
        flags_association = np.zeros(target_num) # the number of targets
        for box in sphd_det_msg.sphd_measurements:
            # clutter flag:
            flags_detection = np.zeros(target_num) 
            detection_range = box.z_range
            detection_bearing = box.z_bearing
            # transfom target position to local robot frame:
            for i in range(target_num):
                # target position in robot frame:
                target_in_robot = np.matmul(np.linalg.inv(map_T_robot), np.array([[TARGETS[i][0]],[TARGETS[i][1]],[1]]))
                target_in_robot = target_in_robot[0:2]
                ground_range = np.linalg.norm(target_in_robot)
                ground_bearing = np.arctan2(target_in_robot[1], target_in_robot[0])
                # within the field of view of the robot:
                if((ground_range >= r_min and ground_range <= r_max) and (np.abs(ground_bearing) <= b_max)):#target_in_robot[0] > 0): 
                    if(box.z_class == "None" or detection_range < r_min):  # detect no objects:
                        # false negative:
                        flag_valid = 1
                        # false negative number:
                        break
                    else:                       # detect objects:
                        # the difference between detection position and ground positon:
                        diff_range = np.abs(ground_range - detection_range)
                        diff_bearing = np.abs(ground_bearing - detection_bearing)
                        rospy.loginfo("The range and bearing difference: " + str(diff_range) + ", " + str(diff_bearing))
                        # measurement-to-target association:
                        if(diff_range <= sigma_r and diff_bearing <= sigma_b):
                            flags_detection[i] = 1 # detect something
                            if(flags_association[i] == 0): # hasn't been associated
                                # set association flag to True: 
                                flags_association[i] = 1
                                # true detection number:
                                flag_valid = 1
                                # bin map:
                                idx_b = int(np.floor((detection_bearing-b_min)/BIN_SIZE_B))
                                if(idx_b == BIN_NUM_B):
                                    idx_b = idx_b - 1
                                idx_r = int(np.floor((detection_range-r_min)/BIN_SIZE_R))
                                if(idx_r >= BIN_NUM_R):
                                    idx_r = BIN_NUM_R - 1#idx_r - 1
                                bin_true[idx_b][idx_r] += 1
                            else:
                                # clutter detection
                                flag_valid = 1
                                # clutter number:
                                cl_cnt += 1
                                # bin map:
                                idx_b = int(np.floor((detection_bearing-b_min)/BIN_SIZE_B))
                                if(idx_b == BIN_NUM_B):
                                    idx_b = idx_b - 1
                                idx_r = int(np.floor((detection_range-r_min)/BIN_SIZE_R))
                                if(idx_r >= BIN_NUM_R):
                                    idx_r = BIN_NUM_R - 1 #idx_r - 1
                                bin_fp[idx_b][idx_r] += 1
                            break 
            # detect clutter:
            if((np.sum(flags_detection) == 0) and box.z_class != "None" and detection_range >= r_min and detection_range <= r_max):  # detect something:
                # clutter detection
                flag_valid = 1
                # clutter number:
                cl_cnt += 1
                # bin map:
                idx_b = int(np.floor((detection_bearing-b_min)/BIN_SIZE_B))
                if(idx_b == BIN_NUM_B):
                    idx_b = idx_b - 1
                idx_r = int(np.floor((detection_range-r_min)/BIN_SIZE_R))
                if(idx_r >= BIN_NUM_R):
                    idx_r = BIN_NUM_R - 1#idx_r - 1
                bin_fp[idx_b][idx_r] += 1
    
        if(flag_valid == 1): # valid measurement set
            clutter_cnt[meas_cnt] = cl_cnt
            meas_cnt += 1
            

    # print informaiton:
    print("measurement_sets_num:", meas_cnt)
    print("bin_fp  :", bin_fp)
    print("bin_true:", bin_true)
    print("clutter_cnt:", clutter_cnt)
    # save to txt:
    np.savetxt("bin_fp.txt", bin_fp, fmt="%s")
    np.savetxt("bin_true.txt", bin_true, fmt="%s")
    np.savetxt("clutter_cnt.txt", clutter_cnt, fmt="%s")
    # close the rosbag:
    bag.close()
    
if __name__ == '__main__':
    clutter_process(SIGMA_RANGE, SIGMA_BEARING)
        
