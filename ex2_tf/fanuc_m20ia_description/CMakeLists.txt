# According to the format from https://github.com/ros/urdf_tutorial/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(fanuc_m20ia_description)

find_package(catkin REQUIRED)
catkin_package()

# Puts directories required by my package (in this case the ones containing meshes and .urdf file)
# into /opt/ros/melodic/share/package_name . This is done when my package is installed as a ROS package.
# From http://docs.ros.org/en/indigo/api/catkin/html/howto/format2/installing_other.html
# "Sometimes your package needs to install additional files, like roslaunch scripts or parameter settings.
# In most cases these data are platform-independent, so install them within your package’s share directory."
foreach(dir meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
