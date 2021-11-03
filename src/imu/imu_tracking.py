import pyrealsense2 as rs
import math
import numpy as np

import time


class TrackingCam:
    """
    This class controls the Intel Realsense Tracking Camera T265
    """

    def __init__(self):
        self.pipeline = 0
        self.config = 0
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)
        self.pose = None
        self.data = None 

    def start_stream(self):
        """
        Starts the stream of the tracking camera. 
        """
        self.pipeline.start(self.config)

    def stop_stream(self):
        """
        Stops the stream of the tracking camera. 
        """
        self.pipeline.stop()

    def receive_data(self, datatypes, turn_off=False):
        """
        Receives the specified data

        :param datatypes: list of datatypes wanted 
        :type datatypes: list of Strings
        :param turn_off: whether or not the camera turns off at receiving data
        :type turn_off: boolean
        :return: list of datatypes specified
        :rtype: list

        """
        data_returned = []
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                self.pose = frames.get_pose_frame()
                if self.pose:
                    self.data = self.pose.get_pose_data()

                    # Receive each datatype 
                    for d in datatypes:
                        if(d == "VELOCITY"):
                            velocity = self.get_velocity()
                            data_returned.append([velocity])
                        elif(d == "ACCELERATION"):
                            acceleration = self.get_acceleration()
                            data_returned.append([acceleration])
                        elif(d == "POSITION"):
                            position = self.get_position()
                            data_returned.append([position])
                        elif(d == "STEREO"):
                            stereo = self.get_stereo
                            data_returned.append([stereo])
                    break
        except:
            print("Error receiving data")
            return None
            
        finally:
            if(turn_off):
                self.stop_stream()
            if(data_returned is []):
                return None
            else:
                return data_returned
        
    def pose_to_ypr(self):
        """
        Converts pose to yaw, pitch and roll
        Use receive_data before this function to update the pose

        """
        try:
            w = self.data.rotation.w
            x = -self.data.rotation.z
            y = self.data.rotation.x
            z = -self.data.rotation.y

            yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi
            pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi
            roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi

        except:
            print("Could not convert pose to Euler angles")
            return None
        return yaw, pitch, roll    

    def get_position(self):
        """
        Returns the position of the device
        :return: position of device in x, y, z 
        :rtype: int, int, int

        """
        position = self.data.translation.x, self.data.translation.y, self.data.translation.z
        return position

    def get_velocity(self):
        """
        Returns the velocity of the device
        :return: velocity of device in x, y, z 
        :rtype: int, int, int

        """
        velocity = self.data.velocity.x, self.data.velocity.y, self.data.velocity.z
        return velocity

    def get_acceleration(self):
        """
        Returns the acceleration of the device
        :return: acceleration of device in x, y, z 
        :rtype: int, int, int

        """
        acceleration = self.data.acceleration.x, self.data.acceleration.y, self.data.acceleration.z 
        return acceleration
