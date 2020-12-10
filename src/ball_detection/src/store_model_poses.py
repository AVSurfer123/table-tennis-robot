#!/usr/bin/env python
import rospy
import rospkg
from rospy_message_converter import message_converter
import time
import yaml
import os
from geometry_msgs.msg import Pose
from get_model_gazebo_pose import GazeboModel

class ModelPoseStore():
    def __init__(self, model_to_track_list):

        # This are the models that we will generate information about.
        self.gz_model_obj = GazeboModel(model_to_track_list)
        
        
    def get_pose_of_model(self, model_name):
        """
        Retrieves the position of an object from the world
        """
        pose_now = self.gz_model_obj.get_model_pose(model_name)
        
        return pose_now
        
    def store_model_poses_for_duration(self, model_name, duration=1.0, frequency_save_pose=1.0, file_to_store="poses.yaml"):
        """
        We store in the Given File Name the poses of the given model, whihc has to be inside the init model list
        inside a Yaml file
        """
        
        
        
        rate = rospy.Rate(frequency_save_pose)
        
        init_time_stamp = rospy.get_time()
        now_time_stamp = rospy.get_time()
        delta_time = now_time_stamp - init_time_stamp
        
        with open(file_to_store, 'w') as outfile:
            
            while delta_time <= duration:
                now_pose = None
                while now_pose == None:
                    now_pose = self.get_pose_of_model(model_name)
                now_dict = 'Position: (' + str(now_pose.position.x) + ', '+ str(now_pose.position.y) + ', '+ str(now_pose.position.z) + ')'
                #now_dict1 = 'Orientation: (' + str(now_pose.orientation.x) + ', '+ str(now_pose.orientation.y) + ', '+ str(now_pose.orientation.z) + ')'
                
                if now_pose.position.y > 0:
                    return
    
                rospy.logdebug(str(now_dict))
                
                yaml.dump(now_dict, outfile, default_flow_style=False)
                #yaml.dump(now_dict1, outfile, default_flow_style=False)
                
                rate.sleep()
                
                now_time_stamp = rospy.get_time()
                delta_time = now_time_stamp - init_time_stamp
                
                rospy.logdebug("TIME SAVING==>"+str(delta_time))
        
 
        
        
    def reformat_pose_to_dict(self, pose):
        """
        Converts Pose to dict
        """
        
        dictionary = message_converter.convert_ros_message_to_dictionary(pose)
        
        now_time_stamp = rospy.get_time()
        
        stamp_dict = {str(now_time_stamp):dictionary}
        
        return stamp_dict
        
    
    

if __name__ == '__main__':
    rospy.init_node('store_model_poses_node', anonymous=True, log_level=rospy.DEBUG)
    
    model_to_save_poses = "ping_pong_ball_0"
    model_to_track_list = [model_to_save_poses]
    model_pose_store_obj = ModelPoseStore(model_to_track_list)
    
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    path_to_package = rospack.get_path('ball_detection')
    pose_files_dir = os.path.join(path_to_package)
    
    
    if not os.path.exists(pose_files_dir):
        os.makedirs(pose_files_dir)
    
    
    pose_file_name = model_to_save_poses+str(time.time())+".yaml"
    pose_file_path = os.path.join(pose_files_dir, pose_file_name)
    
    
    model_pose_store_obj.store_model_poses_for_duration(    model_name=model_to_track_list[0],
                                                            duration=10.0,
                                                            frequency_save_pose=50.0,
                                                            file_to_store=pose_file_path)