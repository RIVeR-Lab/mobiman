#!/usr/bin/env python3

'''
LAST UPDATE: 2023.07.24

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
'''

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from gazebo_conveyor.srv import ConveyorBeltControl
from geometry_msgs.msg import Quaternion, Pose, Point
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class PkgSpawner():

	def __init__(self) -> None:
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('mobiman_simulation') + "/urdf/"

		self.pkgs_ign_name = []
		self.pkgs_ign_path = []
		self.pkgs_man_name = []
		self.pkgs_man_path = []
		
		self.pkgs_ign_name.append("red_cube")
		self.pkgs_ign_path.append(self.path + "red_cube.urdf")
		self.pkgs_ign_name.append("green_cube")
		self.pkgs_ign_path.append(self.path + "green_cube.urdf")
		self.pkgs_ign_name.append("blue_cube")
		self.pkgs_ign_path.append(self.path + "blue_cube.urdf")
		
		self.pkgs_man_name.append("normal_pkg")
		self.pkgs_man_path.append(self.path + "normal_pkg.urdf")
		self.pkgs_man_name.append("long_pkg")
		self.pkgs_man_path.append(self.path + "long_pkg.urdf")
		self.pkgs_man_name.append("longwide_pkg")
		self.pkgs_man_path.append(self.path + "longwide_pkg.urdf")

		self.pkg_ign_index = 0
		self.pkg_man_index = 0

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		self.cc = rospy.ServiceProxy("/conveyor/control", ConveyorBeltControl)

	def checkModelPkg(self, pkg_name):
		res = self.ms(pkg_name, "world")
		return res.success

	def getPkgPosX(self, pkg_name):
		res = self.ms(pkg_name, "world")
		return res.pose.position.x

	def spawnModelPkgIgn(self):
		pkg_ign_path = self.pkgs_ign_path[self.pkg_ign_index]
		pkg_ign_name = self.pkgs_ign_name[self.pkg_ign_index]

		with open(pkg_ign_path, "r") as f:
			pkg_ign_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0, 0, 0) # type: ignore
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=-4.5, y=-2.5, z=0.5), orient)
		
		self.sm(pkg_ign_name, pkg_ign_urdf, '', pose, 'world')
		
		if self.pkg_ign_index < len(self.pkgs_ign_name)-1:
			self.pkg_ign_index += 1
		else:
			self.pkg_ign_index = 0
		rospy.sleep(10.0)

	def spawnModelPkgMan(self):
		pkg_man_path = self.pkgs_man_path[self.pkg_man_index]
		pkg_man_name = self.pkgs_man_name[self.pkg_man_index]

		with open(pkg_man_path, "r") as f:
			pkg_man_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0, 0, 0) # type: ignore
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=-4.5, y=-2.5, z=0.6), orient)
		
		self.sm(pkg_man_name, pkg_man_urdf, '', pose, 'world')
		
		if self.pkg_man_index < len(self.pkgs_man_name)-1:
			self.pkg_man_index += 1
		else:
			self.pkg_man_index = 0
		rospy.sleep(10.0)

	def deleteModelPkg(self, pkg_name):
		self.dm(pkg_name)
		rospy.sleep(0.5)

	def shutdown_hook(self):

		print("pkg_ign_index: " + str(self.pkg_ign_index))

		pkg_ign_name = self.pkgs_ign_name[self.pkg_ign_index]
		if self.checkModelPkg(pkg_ign_name):
			self.deleteModelPkg(pkg_ign_name)
		
		pkg_man_name = self.pkgs_man_name[self.pkg_man_index]
		if self.checkModelPkg(pkg_man_name):
			self.deleteModelPkg(pkg_man_name)

		print("[spawn_packages::shutdown_hook] Shutting down...")

	def startConveyor(self):
		#print("[spawn_packages::startConveyor] Conveyor started!")
		self.cc(10)
		#rospy.sleep(0.5)

	def stopConveyor(self):
		#print("[spawn_packages::stopConveyor] Conveyor stopped!")
		self.cc(0)
		#rospy.sleep(0.5)

if __name__ == "__main__":
	print("[spawn_packages::__main__] Waiting for gazebo services...")
	rospy.init_node("spawn_packages")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	rospy.wait_for_service("/conveyor/control")
	
	## Start Gazebo link attach service
	rospy.loginfo("[spawn_packages::__main__] Creating ServiceProxy to /link_attacher_node/attach")
	attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	attach_srv.wait_for_service()
	rospy.loginfo("[spawn_packages::__main__] Created ServiceProxy to /link_attacher_node/attach")

	## Start Gazebo link detach service
	rospy.loginfo("[spawn_packages::__main__] Creating ServiceProxy to /link_attacher_node/detach")
	detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
	detach_srv.wait_for_service()
	rospy.loginfo(" [spawn_packages::__main__]Created ServiceProxy to /link_attacher_node/detach")

	#r = rospy.Rate(15)
	ps = PkgSpawner()
	
	rospy.on_shutdown(ps.shutdown_hook)
	
	ps.startConveyor()

	while not rospy.is_shutdown():
		
		pkg_ign_name = ps.pkgs_ign_name[ps.pkg_ign_index]
		if ps.checkModelPkg(pkg_ign_name) == False:
			ps.spawnModelPkgIgn()
		elif ps.getPkgPosX(pkg_ign_name) > 5.0:
			ps.deleteModelPkg(pkg_ign_name)

		pkg_man_name = ps.pkgs_man_name[ps.pkg_man_index]
		if ps.checkModelPkg(pkg_man_name) == False:
			ps.spawnModelPkgMan()
		elif ps.getPkgPosX(pkg_man_name) > 5.0:
			ps.deleteModelPkg(pkg_man_name)

		if (ps.checkModelPkg("longwide_pkg")):
			ps.stopConveyor()

		#r.sleep()

	print("[spawn_packages::__main__] END")
		