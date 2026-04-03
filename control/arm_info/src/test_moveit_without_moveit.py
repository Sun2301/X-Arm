#!/usr/bin/env python
import rospy
from arm_info.srv import kinemarics, kinemaricsRequest
from sensor_msgs.msg import JointState
#from Arm_Lib.Arm_Lib import Arm_Device

def move_arm_direct(x, y, z):
    rospy.init_node('direct_arm_control')

    #dofbot = Arm_Device()

    # 1. Setup the Publisher to the motors (Dofbot specific topic)
    # Note: Check if your topic is actually '/common_joint_states'
    #pub = rospy.Publisher('/common_joint_states', JointState, queue_size=10)
    
    # 2. Setup the IK Service Client
    rospy.wait_for_service('/get_kinemarics')
    kin_service = rospy.ServiceProxy('/get_kinemarics', kinemarics)

    # 3. Get the Joint Angles from your service
    request = kinemaricsRequest()
    request.kin_name = "ik"
    request.tar_x = x
    request.tar_y = y
    request.tar_z = z
    # Adjust orientation if needed
    request.Roll = -140.0 

    try:
        response = kin_service(request)
        
        # 4. Create the JointState message to move the servos
        joint_msg = JointState()
        joint_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        # Convert the 0-180 degree response back to radians for the driver
        # Dofbot driver usually expects degrees OR radians depending on the node
        # If your driver expects degrees, use the response directly:
        joint_msg.position = [
            response.joint1, 
            response.joint2, 
            response.joint3, 
            response.joint4, 
            response.joint5
        ]

        # 5. Send the command
        rospy.loginfo("Moving to XYZ: {}, {}, {}".format(x,y,z))
        #pub.publish(joint_msg)
        #dofbot.dofbot.Arm_serial_servo_write6_array(joint_msg.position,500)

    except rospy.ServiceException as e:
        print("Service call failed:".format(e))

if __name__ == "__main__":
    # Example: Move to a point 15cm forward, 10cm up
    move_arm_direct(-0.002, 0.27, 0.105)












"""import rospy
from arm_info.srv import kinemarics, kinemaricsRequest
# Assuming you use the Dofbot's custom message or a standard JointState
from sensor_msgs.msg import JointState 

class DirectCommander:
    def __init__(self):
        rospy.init_node('safe_direct_control')
        self.pub = rospy.Publisher('/common_joint_states', JointState, queue_size=10)
        self.client = rospy.ServiceProxy('/get_kinemarics', kinemarics)
        
    def move_to(self, x, y, z):
        # Step 1: Check Workspace
        if not is_safe(x, y, z): return

        # Step 2: Get IK
        rospy.wait_for_service('/get_kinemarics')
        req = kinemaricsRequest(kin_name="ik", tar_x=x, tar_y=y, tar_z=z)
        res = self.client(req)
        
        joints = [res.joint1, res.joint2, res.joint3, res.joint4, res.joint5]

        # Step 3: Check Joint Limits
        if check_joint_limits(joints):
            msg = JointState()
            msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
            msg.position = joints
            self.pub.publish(msg)
            rospy.loginfo("Command sent safely.")

if __name__ == "__main__":
    commander = DirectCommander()
    commander.move_to(0.15, 0.0, 0.12)"""
