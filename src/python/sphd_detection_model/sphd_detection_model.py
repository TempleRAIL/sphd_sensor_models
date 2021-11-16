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
SIGMA_RANGE = 3*0.25   # 3*r, r: 0.25
SIGMA_BEARING = 3*(np.pi/180)*3 # 3*b*pi/180, b: 3 degrees

# read rosbag and process the measurements:
def detection_process(sigma_r, sigma_b):
    # detection model information:
    BIN_NUM = 14
    BIN_SIZE = 0.2
    # bearing:
    b_max = np.pi/4
    b_min = -np.pi/4
    # range:
    r_max = 3.1
    r_min = 0.4
    # true detection number in the range bin: 0.4~3.1m / 0.2 = 14
    bin_true = np.zeros(BIN_NUM)
    # false negative number in the range bin: 0.4~3.1m / 0.2 = BIN_NUM
    bin_fn = np.zeros(BIN_NUM)
    # valid measurement number:
    meas_cnt = 0

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
        # association flag:
        flags_association = np.zeros(target_num) # the number of targets
        for box in sphd_det_msg.sphd_measurements:
            # cluster flag:
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
                        # bin map:
                        idx = int(np.floor((ground_range-r_min)/BIN_SIZE))
                        if(idx == BIN_NUM):
                            idx = idx - 1
                        bin_fn[idx] += 1
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
                                idx = int(np.floor((ground_range-r_min)/BIN_SIZE))
                                if(idx == BIN_NUM):
                                    idx = idx - 1
                                bin_true[idx] += 1
                            else:
                                # cluster detection
                                flag_valid = 1
                            break 
            # detect cluster:
            if((np.sum(flags_detection) == 0) and box.z_class != "None" and detection_range >= r_min):  # detect something:
                # cluster detection
                flag_valid = 1
                # cluster number:
    
        if(flag_valid == 1): # valid measurement set
            meas_cnt += 1

    # print informaiton:
    print("measurement_sets_num:", meas_cnt)
    print("bin_fn  :", bin_fn)
    print("bin_true:", bin_true)
    # save to txt:
    np.savetxt("bin_fn.txt", bin_fn, fmt="%s")
    np.savetxt("bin_true.txt", bin_true, fmt="%s")
    # close the rosbag:
    bag.close()
    
if __name__ == '__main__':
    detection_process(SIGMA_RANGE, SIGMA_BEARING)
        
