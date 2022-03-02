from math import cos,sin
import numpy as np
import sys
import pdb

#######################################
## PREDIFINE CONSTATNS OF THE EXERCISE

ROLL=2.5; PITCH=5; YAW=60 
TRANSLATION = np.array([0, 0, 0.25])
######################################


def ENU_to_NED(X):
	# Perform the rotation from Global World Coordinates system to INS
	rotation_matrix = _3D_rotation_matrix(ROLL, PITCH, YAW)
	return rotation_matrix @ X

def NED_to_ENU(X):
	# Perform the rotation from INS to Global World Coordinates system.
	# Inverse of the previous one
	rotation_matrix = _3D_rotation_matrix(ROLL, PITCH, YAW, inverso=True)
	return rotation_matrix @ X
	
	
def _3D_rotation_matrix(roll, pitch, yaw, order='ypr', inverso=False):
	# Return the rotation matrix in 3D coordinate systems
	yaw = np.array([
		[cos(yaw), -sin(yaw), 0],
		[sin(yaw), cos(yaw), 0],
		[0, 0, 1]
	])
	pitch = np.array([
		[cos(pitch), 0, sin(pitch)],
		[0, 1, 0],
		[-sin(pitch), 0, cos(pitch)]
	])
	roll = np.array([
		[1, 0, 0],
		[0, cos(roll), -sin(roll)],
		[0, sin(roll), cos(roll)]
	]) 

	rotation_matrix = None
	if order=='ypr':
		if inverso:
			rotation_matrix = yaw.T @ pitch.T @ roll.T
		else:
			rotation_matrix = roll @ pitch @ yaw
	elif order == 'rpy':
		if inverso:
			rotation_matrix = roll.T @ pitch.T @ yaw.T
		else:
			rotation_matrix = yaw @ pitch @ roll
	else:
		print("Order not supported")
		sys.exit(-1)

	return rotation_matrix
		

def _3D_Transform(rotation, T):
	# Perform the translation in 3D coordinate systems 
	transformation_matrix = np.zeros((4,4))
	transformation_matrix[0:3,0:3] = rotation*1
	transformation_matrix[:-1,-1] = T
	transformation_matrix[-1,-1] = 1
	return transformation_matrix
	
	
def camera3D_to_gimbal3D(X, transformation_Matrix):
	# Perform the transformation from camera (nadiral orientation) 
	# to gimbal (forward orientation) 
	return 

def gimbal3D_to_INS3D(X, transformation_Matrix):
	# Perform the transformation from camera (nadiral orientation) 
	# to gimbal (forward orientation) 
	return 
	
def compose_camera3D_to_World3D(transformation_Matrices):
	# Perform the transformation from camera (nadiral orientation) 
	# to gimbal (forward orientation) 
	return 	
def compose_World3D_to_camera3D(X, transformation_Matrix):
	# Perform the transformation from camera (nadiral orientation) 
	# to gimbal (forward orientation) 
	return 	
	

def main(args):
    return 0

if __name__ == '__main__':
    
	# Pruebas
	global_ = np.array([1,2,3])
	print("Global:", global_)
	
	local_ = ENU_to_NED(global_)
	print("Local:", local_)
	
	global_2 = NED_to_ENU(local_)
	print("Global2:", global_2)
	sys.exit(main(sys.argv))
