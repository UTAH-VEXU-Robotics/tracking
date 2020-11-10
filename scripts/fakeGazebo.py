#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
from tracking.msg import Model, Models
import rospy
import time

def main():
    print("fake gazebo")
    rospy.init_node('fake_gazebo')

    main.modelPub = rospy.Publisher('/field/new_models', Models, queue_size=5)

    main.models = Models()

    def getCallback(models):
        main.models = models

    def setCallback(imodel):
        for model in main.models.models:
            if(model.name == imodel.name):
                print("changed model: " + model.name)
                model = imodel

    rospy.Subscriber("/gazebo/set_field", Model, setCallback)
    rospy.Subscriber("/gazebo/get_field", Models, getCallback)

    while not rospy.is_shutdown():
        try:
            time.sleep(.25)

            if(main.models != Models()):
                main.modelPub.publish(main.models)

        except rospy.ROSInterruptException:
            print("failed to display models")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
