#! /usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import tf
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv
import sys
import os

g_is_record_corver  = False
g_cmd_finish_record = False
g_file_path = ''
g_fd = ''
g_freq = 20
def  read_path_from_local_to_global_planner(file_path,topic = '/move_base_simple/fix_plan'):
    pub = rospy.Publisher(topic, Path, queue_size=0)
    if(os.path.exists(file_path) == False):
        return False
    fd = open(file_path)
    csv_reader = csv.reader(fd)
    pathMsg = Path()
    pathMsg.header.frame_id = 'map'
    pathMsg.header.stamp = rospy.get_rostime()
    for row in csv_reader:
        pose_ = PoseStamped()
        pose_.header.frame_id = 'map'
        pose_.header.stamp = rospy.get_rostime()
        pose_.pose.position.x = float(row[0])
        pose_.pose.position.y = float(row[1])
        pose_.pose.position.z = float(row[2])
        pose_.pose.orientation.x = float(row[3])
        pose_.pose.orientation.y = float(row[4])
        pose_.pose.orientation.z = float(row[5])
        pose_.pose.orientation.w = float(row[6])
        pathMsg.poses.append(pose_)
    fd.close()
    pub.publish(pathMsg)
    return True
def ros_cmd_callback(data):
    global g_cmd_finish_record
    if(data.data == 'stop_record'):
        g_cmd_finish_record = True
        print 'record path file recv the cmd is :',data.data,' .'

def ros_move_base_status_callback(data):
    state_num = len(data.status_list) - 1
    if state_num < 0:
        state_num = 0;
    status_id = data.status_list[state_num].status
    state_tabl = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                  'RECALLED', 'LOST']
    # print 'record_file recv the status :', state_tabl[status_id],' .'

def is_need_record(var):
    position_tolerate = 0.05
    yaw_tolerate = 0.01
    a1 = math.fabs(var['new_trans'][0] - var['last_trans'][0])
    a2 = math.fabs(var['new_trans'][1] - var['last_trans'][1])
    a3 = math.fabs(var['new_trans'][2] - var['last_trans'][2])
    a4 = math.fabs(var['new_rpy'][0] - var['last_rpy'][0])
    a5 = math.fabs(var['new_rpy'][1] - var['last_rpy'][1])
    a6 = math.fabs(var['new_rpy'][2] - var['last_rpy'][2])
    dis_trans = math.sqrt(a1 ** 2 + a2 ** 2)
    if(dis_trans > position_tolerate) or (a6 > yaw_tolerate):
        return True
    else:
        return False

def record_new_pose(writer,tmp):
    tmp1 =  list(tmp['new_trans']) + list(tmp['new_quantion']);
    for row in [tmp1]:
        writer.writerow(row)
        # print 'record the path write success ',row
    return 1

def get_new_file_path(file_path):
    folder_path = os.path.dirname(file_path)
    file_name = file_path.split('/')[-1]
    (file_name, exec_name) = os.path.splitext(file_name)
    # print file_name ,exec_name
    if (os.path.exists(file_path) == True):
        for i in range(1, 1000, 1):
            file_name_tmp = file_name + '_' + str(i) + exec_name
            new_file_path = folder_path + '/' + file_name_tmp
            if (os.path.exists(new_file_path) == False):
                return new_file_path
    else:
        return file_path
def listen(file_path):

    global  g_cmd_finish_record
    global  g_is_record_corver
    global  g_freq
    path = file_path
    freq = g_freq
    last_trans = [0, 0, 0]
    new_trans = [0, 0, 0]
    new_quantion = [0, 0, 0, 0]
    last_quantion = [0, 0, 0, 0]
    new_rpy = [0, 0, 0]
    last_rpy = [0, 0, 0]
    last_trans = [0,0,0]

    # rospy.Subscriber("/move_base/status", GoalStatusArray, ros_move_base_status_callback)
    rospy.Subscriber("/record/goal", String, ros_cmd_callback)
    listener = tf.TransformListener()
    rospy.logout('Begin to record the path.')

    if(g_is_record_corver == True):
        fd = open(file_path,'w+')
        csv_writer = csv.writer(fd)
    else:
        new_file_path = get_new_file_path(file_path)
        file_path = new_file_path
        fd = open(new_file_path, 'a+')
        csv_writer = csv.writer(fd)

    rate1 = rospy.Rate(freq)

    while not rospy.is_shutdown():
        try:
            (new_trans_tmp, new_quaternion_tmp) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            new_quaternion = list(new_quaternion_tmp)
            new_trans =new_trans_tmp
            new_rpy = tf.transformations.euler_from_quaternion(new_quaternion)
            tmp = {
                "new_trans":new_trans,
                "last_trans":last_trans,
                "new_rpy":new_rpy,
                "last_rpy":last_rpy,
                "new_quantion":new_quaternion_tmp,
                "last_quantion":last_quantion
                 }
            if(is_need_record(tmp)):
                record_new_pose(csv_writer,tmp)
                last_trans = new_trans
                last_quantion = new_quantion
                last_rpy = new_rpy
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if (g_cmd_finish_record == True):
            fd.close()
            g_cmd_finish_record = False
            print 'finish the record the path.'
            print 'the file path is : ',file_path
            break
        try:
            rate1.sleep()
        except(rospy.exceptions.ROSInterruptException):
            continue
if __name__ == '__main__':

    argv_len = len(sys.argv)
    if(argv_len <= 1):
        mode = 'record'
        file_path = "./data.csv"
        topic = '/move_base_simple/fix_plan'
    elif(argv_len == 3):
        mode = sys.argv[1]
        file_path = sys.argv[2]
        topic = '/move_base_simple/fix_plan'
    elif(argv_len == 4):
        mode = sys.argv[1]
        file_path = sys.argv[2]
        topic = sys.argv[3]

    rospy.init_node('record_path', anonymous=True)
    if( mode == 'record'):
        listen(file_path)
    elif(mode == 'read'):
        rate1 = rospy.Rate(3)
        i = 2;
        while not rospy.is_shutdown() and i:
            i = i-1
            if(read_path_from_local_to_global_planner(file_path,topic) == False):
                print 'send the record falied '
            else:
                print 'send successfully'
            try:
                rate1.sleep()
            except(rospy.exceptions.ROSInterruptException):
                continue


