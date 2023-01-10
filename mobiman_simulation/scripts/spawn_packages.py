#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from gazebo_conveyor.srv import ConveyorBeltControl
from geometry_msgs.msg import Quaternion, Pose, Point

class PkgSpawner():

	def __init__(self) -> None:
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('mobiman_simulation') + "/urdf/"
		self.pkgs_ign = []
		self.pkgs_man = []
		
		self.pkgs_ign.append(self.path + "red_cube.urdf")
		self.pkgs_ign.append(self.path + "green_cube.urdf")
		self.pkgs_ign.append(self.path + "blue_cube.urdf")

		self.pkgs_man.append(self.path + "normal_pkg.urdf")
		self.pkgs_man.append(self.path + "long_pkg.urdf")
		self.pkgs_man.append(self.path + "longwide_pkg.urdf")

		self.pkg_ign_index = 0
		self.pkg_man_index = 0

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		self.cc = rospy.ServiceProxy("/conveyor/control", ConveyorBeltControl)

	def checkModelPkgIgn(self):
		res = self.ms("pkg_ign", "world")
		return res.success
	
	def checkModelPkgMan(self):
		res = self.ms("pkg_man", "world")
		return res.success

	def getPosZPkgIgn(self):
		res = self.ms("pkg_ign", "world")
		return res.pose.position.z

	def getPosZPkgMan(self):
		res = self.ms("pkg_man", "world")
		return res.pose.position.z

	def spawnModelPkgIgn(self):
		pkg_ign = self.pkgs_ign[self.pkg_ign_index]

		with open(pkg_ign, "r") as f:
			pkg_ign_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0, 0, 0)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=-4.5, y=-2.5, z=1.0), orient)
		
		self.sm("pkg_ign", pkg_ign_urdf, '', pose, 'world')
		
		if self.pkg_ign_index < len(self.pkgs_ign)-1:
			self.pkg_ign_index += 1
		else:
			self.pkg_ign_index = 0
		rospy.sleep(1.0)

	def spawnModelPkgMan(self):
		pkg_man = self.pkgs_man[self.pkg_man_index]

		with open(pkg_man, "r") as f:
			pkg_man_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0, 0, 0)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=-4.5, y=-2.5, z=1.0), orient)
		
		self.sm("pkg_man", pkg_man_urdf, '', pose, 'world')
		
		if self.pkg_man_index < len(self.pkgs_man)-1:
			self.pkg_man_index += 1
		else:
			self.pkg_man_index = 0
		#rospy.sleep(0.5)

	def deleteModelPkgIgn(self):
		self.dm("pkg_ign")
		rospy.sleep(0.5)

	def deleteModelPkgMan(self):
		self.dm("pkg_man")
		rospy.sleep(0.5)

	def shutdown_hook(self):
		self.deleteModelPkgIgn()
		self.deleteModelPkgMan()
		print("[spawn_packages::shutdown_hook] Shutting down...")

	def startConveyor(self):
		self.cc(100)
		#rospy.sleep(0.5)

if __name__ == "__main__":
	print("Waiting for gazebo services...")
	rospy.init_node("spawn_packages")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	rospy.wait_for_service("/conveyor/control")
	
	r = rospy.Rate(15)
	ps = PkgSpawner()
	
	rospy.on_shutdown(ps.shutdown_hook)
	
	ps.startConveyor()

	while not rospy.is_shutdown():
		if ps.checkModelPkgIgn() == False:
			ps.spawnModelPkgIgn()
		elif ps.getPosZPkgIgn() < 0.2:
			ps.deleteModelPkgIgn()

		if ps.checkModelPkgMan() == False:
			ps.spawnModelPkgMan()
		elif ps.getPosZPkgMan() < 0.2:
			ps.deleteModelPkgMan()

		r.sleep()
		