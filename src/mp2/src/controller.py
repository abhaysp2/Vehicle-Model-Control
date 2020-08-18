import math
import numpy as np

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive




class bicycleModel():

    def __init__(self):

        """ Series of provided waypoints """
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()
        pt4 = ModelState()
        pt5 = ModelState()
        pt6 = ModelState()
        pt7 = ModelState()

        pt1.pose.position.x = 10
        pt1.pose.position.y = 0
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25

        pt3.pose.position.x = -10
        pt3.pose.position.y = -10
        pt3.twist.linear.x = .25
        pt3.twist.linear.y = .25

        pt4.pose.position.x = 10
        pt4.pose.position.y = -10
        pt4.twist.linear.x = .25
        pt4.twist.linear.y = .25

        pt5.pose.position.x = -10
        pt5.pose.position.y = -10
        pt5.twist.linear.x = .25
        pt5.twist.linear.y = .25

        pt6.pose.position.x = -10
        pt6.pose.position.y = 0
        pt6.twist.linear.x = .25
        pt6.twist.linear.y = .25

        pt7.pose.position.x = 0
        pt7.pose.position.y = 0
        pt7.twist.linear.x = .25
        pt7.twist.linear.y = .25


        self.waypointList = [pt1, pt2, pt3, pt4, pt5, pt6, pt7]


        #self.waypointList = []

        self.length = 1.88

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)


        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)



    def getModelState(self):
        """
            Description:
                Requests the current state of the polaris model when called

            Returns:
                modelState: contains the current model state of the polaris vehicle in gazebo
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState


    def rearWheelModel(self, ackermannCmd):
        """
            Description:
                Contains the mathematical model that will represent the vehicle in gazebo

            Inputs:
                ackermannCmd (AckermannDrive): contains desired vehicle velocity and steering angle velocity
                                               that the model should follow

            Returns:
                A List containing the vehicle's x velocity, y velocity, and steering angle velocity
        """
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        ## TODO: Compute Bicyle Model
        #http://docs.ros.org/diamondback/api/gazebo/html/msg/ModelState.html
        #http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html
        yaw = self.quaternion_to_euler(currentModelState.pose.orientation.x,currentModelState.pose.orientation.y,currentModelState.pose.orientation.z,currentModelState.pose.orientation.w)[2]
        x = ackermannCmd.speed * math.cos(yaw)
        y = ackermannCmd.speed * math.sin(yaw)
        return [x, y, ackermannCmd.steering_angle_velocity]    ## TODO: Return x velocity, y velocity, and steering angle velocity


    def rearWheelFeedback(self, currentState, targetState):
        """
            Description:
                Feedback loop which drives the vehicles to the current waypoint

            Inputs:
                currentState (ModelState): The curret state of the vehicle in gazebo
                targetState  (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                ackermannCmd (AckermannDrive): Will be used to compute the new x,y, and steering angle
                                               velocities of the model
        """

        targetVel = math.sqrt((targetState.twist.linear.x*targetState.twist.linear.x) + ((targetState.twist.linear.y*targetState.twist.linear.y)))
        targetAngVel = targetState.twist.angular.z

        ## TODO: Compute Error to current waypoint
        targetYaw = self.quaternion_to_euler(targetState.pose.orientation.x,targetState.pose.orientation.y,targetState.pose.orientation.z,targetState.pose.orientation.w)[2]
        targetX = targetState.pose.position.x
        targetY = targetState.pose.position.y

        currentYaw = self.quaternion_to_euler(currentState.pose.orientation.x,currentState.pose.orientation.y,currentState.pose.orientation.z,currentState.pose.orientation.w)[2]
        currentX = currentState.pose.position.x
        currentY = currentState.pose.position.y

        a = np.array([[math.cos(currentYaw),math.sin(currentYaw),0],[-math.sin(currentYaw),math.cos(currentYaw),0],[0,0,1]])
        b = np.array([[targetX-currentX],[targetY-currentY],[targetYaw-currentYaw]])
        error_vector = np.dot(a,b) #gives dot product


        ## TODO: Create new AckermannDrive message to return
        # Best values so far: 1, 3, 3
        k1 = 1.5
        k2 = 5
        k3 = 1
        currentVel = targetVel * math.cos(error_vector[2]) + k1*error_vector[0]
        currentAngVel = targetAngVel + (targetVel)*(k2*error_vector[1] + k3*math.sin(error_vector[2]))
        ack_msg = AckermannDrive()
        ack_msg.speed = currentVel
        ack_msg.steering_angle_velocity = currentAngVel

        return ack_msg

    def setModelState(self, currState, targetState):
        """
            Description:
                Sets state of the vehicle in gazebo.

                This function is called by mp2.py at a frequency of apporximately 100Hz

            Inputs:
                currState   (ModelState): The curret state of the vehicle in gazebo
                targetState (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                None
        """

        ## TODO: call controller and model functions
        ack_msg = self.rearWheelFeedback(currState, targetState)
        x_vel, y_vel, steering_vel = self.rearWheelModel(ack_msg)


        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = currState.pose
        newState.twist.linear.x = x_vel             # TODO: Add x velocity
        newState.twist.linear.y = y_vel             # TODO: Add y velocity
        newState.twist.angular.z = steering_vel           # TODO: Add steering angle velocity
        self.modelStatePub.publish(newState)

    def quaternion_to_euler(self, x, y, z, w):
        """
            Description:
                converts quaternion angles to euler angles. Note: Gazebo reports angles in quaternion format

            Inputs:
                x,y,z,w:
                    Quaternion orientation values

            Returns:
                List containing the conversion from quaternion to euler [roll, pitch, yaw]
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]


    def __waypointHandler(self, data):
        """
            Description:
                Callback handler for the /gem/waypoint topic. If a waypoint is published to
                this topic, this function will be called and append the published waypoint to
                the waypoint list.

            Inputs:
                data (ModelState): the desired state of the model

            Returns:
                None
        """
        self.waypointList.append(data)
