import cv2
import numpy as np
import yaml
import rospy


'''
Augmenter class was adapted from
GitHub Repository exA-3 by GitHub user Coral79
Link: https://github.com/Coral79/exA-3/blob/44adf94bad728507608086b91fbf5645fc22555f/packages/augmented_reality_basics/include/augmented_reality_basics/augmented_reality_basics.py
''' 
class Augmenter():
    def __init__(self):
        self.veh = rospy.get_param("~veh")
        
        camera_extrinsic_dict =  self.readYamlFile(f'/data/config/calibrations/camera_extrinsic/{self.veh}.yaml')
        homography = camera_extrinsic_dict['homography']
        
        self.H = [homography[0:3], homography[3:6], homography[6:9]]
        self.Hinv = np.linalg.inv(self.H)
        
        camera_intrinsic_dict =  self.readYamlFile(f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml')
        
        self.K = np.array(camera_intrinsic_dict["camera_matrix"]["data"]).reshape((3, 3))
        self.R = np.array(camera_intrinsic_dict["rectification_matrix"]["data"]).reshape((3, 3))
        self.D = np.array(camera_intrinsic_dict["distortion_coefficients"]["data"])
        self.P = np.array(camera_intrinsic_dict["projection_matrix"]["data"]).reshape((3, 4))
        self.h = camera_intrinsic_dict["image_height"]
        self.w = camera_intrinsic_dict["image_width"]
        
        """
        map_file_dict =  self.readYamlFile('../map/hud.yaml') # TODO update
        
        self.points = map_file_dict["points"]
        self.segments = map_file_dict["segments"]
        """
        self.points = rospy.get_param('~points')
        self.segments = rospy.get_param('~segments')

    def process_image(self, cv_image_raw):

        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), 0, (self.w, self.h))
        res = cv2.undistort(cv_image_raw, self.K, self.D, None, newcameramtx)       
        #res = cv2.fisheye.undistortImage(cv_image_raw, self.K, self.D,self.R, self.K)
        return res

    def ground2pixel(self, point):
        if point[2]!= 0:
            msg = 'This method assumes that the point is a ground point (z=0). '
            msg += 'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
            raise ValueError(msg)

        ground_point = np.array([point[0], point[1], 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]

        pixel = image_point[0:2]
        pixel = np.round(pixel).astype(int)
        print (pixel, image_point)
        return pixel

    def render_segments(self, img):
        for i in range(len(self.segments)):
            point_x = self.points[self.segments[i]["points"][0]][1]
            point_y = self.points[self.segments[i]["points"][1]][1]
            point_x = self.ground2pixel(point_x)
            point_y = self.ground2pixel(point_y)
            color = self.segments[i]["color"]
            
            img = self.draw_segment(img, point_x, point_y, color)
            
        return img

    def draw_segment(self, image, pt_x, pt_y, color):
        """
        source: https://docs.duckietown.org/daffy/duckietown-classical-robotics/out/cra_basic_augmented_reality_exercise.html section 3.5
        """
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0, 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}

        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_x[1]), (pt_y[0], pt_y[1]), (b * 255, g * 255, r * 255), 5)

        return image
    
    def readYamlFile(self,fname):
        """
        source: https://docs.duckietown.org/daffy/duckietown-classical-robotics/out/cra_basic_augmented_reality_exercise.html section 3.5
        
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return
