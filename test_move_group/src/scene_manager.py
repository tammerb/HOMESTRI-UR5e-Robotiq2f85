#! /usr/bin/env python3

import rospkg
import rospy
import xacro
import gazebo_msgs.srv
import os



objects = [
    {
        'urdf': 'parts/box.xacro',
        'model_name': 'box1',
        'color' : '255 0 0 1',
        'position': (0.3, 0.2, 0),
    },
    {
        'urdf': 'parts/box.xacro',
        'model_name': 'box2',
        'color' : '0 255 0 1',
        'position': (0.4, 0.2, 0),
    },
    {
        'urdf': 'parts/box.xacro',
        'model_name': 'box3',
        'color' : '0 0 255 1',
        'position': (0.5, 0.2, 0),
    },
    {
        'urdf': 'parts/plastic_bin.xacro',
        'model_name': 'bin1',
        'color' : '0 153 0 1',
        'position': (0.4, -0.2, 0),
    }
]

class SceneManager:
    def __init__(self, package_name):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package_name)

        self.reference_frame = 'world'

        for object in objects:
            self.spawn_model(**object)
    
    def spawn_model(self, urdf, model_name, color, position):
        xml = xacro.process_file(
            os.path.join(self.package_path, 'urdf', urdf),
            mappings={'color': color}
        )
        xml_str = xml.toxml()

        req = gazebo_msgs.srv.SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = xml_str
        req.initial_pose.position.x = position[0]
        req.initial_pose.position.y = position[1]
        req.initial_pose.position.z = position[2]
        req.initial_pose.orientation.x = 0
        req.initial_pose.orientation.y = 0
        req.initial_pose.orientation.z = 0
        req.initial_pose.orientation.w = 1
        req.reference_frame = self.reference_frame

        self.delete_model(model_name, ignore_err=True)

        self.gazebo_service(
            '/gazebo/spawn_urdf_model',
            gazebo_msgs.srv.SpawnModel,
            req
        )

    def delete_model(self, model_name, ignore_err=False):
        req = gazebo_msgs.srv.DeleteModelRequest()
        req.model_name = model_name
        self.gazebo_service(
            '/gazebo/delete_model',
            gazebo_msgs.srv.DeleteModel,
            req,
            ignore_err=ignore_err
        )

        

    def gazebo_service(self, service_name, service_type, req, ignore_err=False):
        try:
            rospy.wait_for_service(service_name)
            client = rospy.ServiceProxy(service_name, service_type)
            resp = client(req)
            if not resp.success and not ignore_err:
                raise rospy.ServiceException(resp.status_message)
        except rospy.ServiceException as e:
            print("Gazebo Service call failed: %s"%e)

        client.wait_for_service()
        rospy.sleep(1)
        
        
if __name__ == '__main__':
    rospy.init_node('scene_manager')
    scene_manager = SceneManager('homestri_robot_description')
    rospy.spin()

    




        
