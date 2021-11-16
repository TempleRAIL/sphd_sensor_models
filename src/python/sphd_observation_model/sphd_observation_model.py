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
SIGMA_RANGE = 3*np.linspace(0.025, 0.5, num=20)   # 3*r, r: 0~0.5 m: interval:0.05 m 
SIGMA_BEARING = 3*(np.pi/180)*np.linspace(0.5, 10, num=20) # 3*b*pi/180, b:0~10 degree: interval: 1 degree

# read rosbag and process the measurements:
def measurement_process(sigma_r, sigma_b):
    # measurement model information:
    # bearing:
    b_max = np.pi/4
    b_min = -np.pi/4
    # range:
    r_max = 3.1
    r_min = 0.4
    # true detection number:
    tr_cnt = 0
    # false negtive number:
    fn_cnt = 0
    # cluster number:
    cl_cnt = 0 
    # sum sqaure error:
    square_error = 0
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
                        # false negative number:
                        fn_cnt += 1
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
                                tr_cnt += 1
                                # sum square error:
                                square_error +=  diff_range**2 + diff_bearing**2
                            else:
                                # cluster detection
                                # cluster number:
                                cl_cnt += 1 
                            break 
            # detect cluster:
            if((np.sum(flags_detection) == 0) and box.z_class != "None" and detection_range >= r_min):  # detect something:
                # cluster detection
                # cluster number:
                cl_cnt += 1  

    # print informaiton:
    print("true detection number:", tr_cnt)
    print("false negative number:", fn_cnt)
    print("cluster number:", cl_cnt)
    print("sum square error:", square_error)
    # close the rosbag:
    bag.close()
    
    return tr_cnt, fn_cnt, cl_cnt, square_error

# grid search:
def grid_search():
    # measurement model information:
    # true detection number:
    tr_cnt = np.zeros((len(SIGMA_RANGE), len(SIGMA_BEARING)))
    # false negtive number:
    fn_cnt = np.zeros((len(SIGMA_RANGE), len(SIGMA_BEARING)))
    # cluster number:
    cl_cnt = np.zeros((len(SIGMA_RANGE), len(SIGMA_BEARING)))
    # sum sqaure error:
    square_error = np.zeros((len(SIGMA_RANGE), len(SIGMA_BEARING)))
    # gird search:
    for r in range(len(SIGMA_RANGE)):
        for b in range(len(SIGMA_BEARING)):
            [tr_cnt[r,b], fn_cnt[r,b], cl_cnt[r,b], square_error[r,b]] \
            = measurement_process(SIGMA_RANGE[r], SIGMA_BEARING[b])
    
    np.savetxt("tr_cnt.txt", tr_cnt, fmt="%s")
    np.savetxt("fn_cnt.txt", fn_cnt, fmt="%s")
    np.savetxt("cl_cnt.txt", cl_cnt, fmt="%s")
    np.savetxt("square_error.txt", square_error, fmt="%s")
        
if __name__ == '__main__':
    grid_search()
        
