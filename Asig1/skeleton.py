# Initial configuration
import numpy as np
from math import cos,sin
import pdb


#######################################
## PREDIFINE CONSTATNS OF THE EXERCISE

ROLL=2.5; PITCH=5; YAW=60 
TRANSLATION = np.array([0, 0, 0.25])
######################################


def enu_to_ned_ned_to_enu():
    transformation_matrix = np.array([
        [0., 1., 0., 0.],
        [1., 0., 0., 0.],
        [0., 0., -1., 0.],
		[0., 0., 0., 1.]
    ])
    return transformation_matrix


def _3D_rotation_matrix(roll, pitch, yaw, order='ypr', inverso=False):
	# Return the rotation matrix in 3D coordinate systems
	yaw = np.array([
		[cos(yaw), -sin(yaw), 0, 0],
		[sin(yaw), cos(yaw), 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])
	pitch = np.array([
		[cos(pitch), 0, sin(pitch), 0.],
		[0, 1, 0, 0],
		[-sin(pitch), 0, cos(pitch), 0],
		[0, 0, 0, 1]
	])
	roll = np.array([
		[1, 0, 0, 0],
		[0, cos(roll), -sin(roll), 0],
		[0, sin(roll), cos(roll), 0],
		[0, 0, 0, 1]
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

	return rotation_matrix



def wcs_to_ins():
    # 1) ENU to NED
    t1 = enu_to_ned_ned_to_enu()

    # 2) yaw, pitch, roll
    t2 = _3D_rotation_matrix(ROLL, PITCH, YAW)
    return t2 @ t1



def ins_to_gimbal(traslation_vector):
    # 1) NED to ENU
    transformation_matrix = enu_to_ned_ned_to_enu()
    
    # 2) Traslacion
    transformation_matrix[:-1,-1] = traslation_vector*1
    pdb.set_trace()
    return transformation_matrix



# Si camara Nadiral orientation -> rotacion 180 en eje e
def gimbal_to_camera(omega=180):
    rotation_mat = np.array([
		[1, 0, 0, 0],
		[0, cos(omega), -sin(omega), 0],
		[0, sin(omega), cos(omega), 0],
		[0, 0, 0, 1]
	]) 

    return rotation_mat



t1 = wcs_to_ins()
print("wcs to ins:", t1)


t2 = ins_to_gimbal(TRANSLATION)
print("ins to gimbal", t2)

exit()

t3 = gimbal_to_camera()
print("gimbal to camera",t3)

# TOTAL
t = t3 @ t2 @ t1
print("full matrix:", t)

### Ejemplo de proyeccion
punto = np.array([3, 4, 0, 1])
proyeccion = t @ punto
print("Proyeccion del punto", punto, "a ", proyeccion)