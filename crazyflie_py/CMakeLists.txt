cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_py)
find_package(catkin REQUIRED COMPONENTS
  rospy
  crazyflie_driver
)

#catkin_package(
#        CATKIN_DEPENDS
#        rospy
#        crazyflie_driver
#        )

catkin_python_setup()
catkin_package()
