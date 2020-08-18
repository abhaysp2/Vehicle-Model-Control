import math
import time
import matplotlib.pyplot as plt

import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel


if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()

    endList = 0;
    xCoord = []
    yCoord = []
    waypoint_plot = []
    #wait till a waypoint is received
    while(not model.waypointList):
        pass
        rospy.sleep(7)

    targetState = ModelState()
    targetState = model.waypointList.pop(0)


    start = time.time()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        currState =  model.getModelState()
        if not currState.success:
            continue


        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)
        xCoord.append(currState.pose.position.x)
        yCoord.append(currState.pose.position.y)


        if(distToTargetX < 1 and distToTargetY < 1):
            if not model.waypointList:
                newState = ModelState()
                newState.model_name = 'polaris'
                newState.pose = currState.pose
                newState.twist.linear.x = 0
                newState.twist.linear.y = 0
                newState.twist.angular.z = 0
                model.modelStatePub.publish(newState)
                #only print time the first time waypontList is empty
                if(not endList):
                    endList = 1
                    end = time.time()
                    print("Time taken:", end-start)

                    waypoint_plot.append(len(xCoord)-1)
                    plt.plot(xCoord, yCoord, '-ro', markevery=waypoint_plot)
                    plt.suptitle('Trajectory of Polaris', fontsize=20)
                    plt.xlabel('x-axis')
                    plt.ylabel('y-axis')
                    plt.show()
            else:
                if(endList):
                    start = time.time()
                    endList = 0
                targetState = model.waypointList.pop(0)
                waypoint_plot.append(len(xCoord)-1)
                markerState = ModelState()
                markerState.model_name = 'marker'
                markerState.pose = targetState.pose
                model.modelStatePub.publish(markerState)
        else:
            model.setModelState(currState, targetState)

            print(distToTargetX,distToTargetY)
            markerState = ModelState()
            markerState.model_name = 'marker'
            markerState.pose = targetState.pose
            model.modelStatePub.publish(markerState)




    rospy.spin()
