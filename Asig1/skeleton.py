# Initial configuration
import numpy as np
from math import cos,sin
import pdb

#######################################
## PREDIFINE CONSTATNS OF THE EXERCISE

ROLL=2.5; PITCH=5; YAW=60 
TRANSLATION = np.array([0, 0, 0.25])

#F = 2406.1
F = 3.62 # focal length (mm)
CX = 25.498; CY=-33.0153
K1 = 0.00247448; K2 = -0.0240244; K3 = 0.0604812; K4 = -0.0275491; 
P1 = 0.000150734; P2 = -0.000271032
B1 = -5.67637; B2 = 0.339356
PIXEL_SIZE = 0.00156  # (mm)
RESOLUTION_W = 4000   # (px)
RESOLUTION_H = 3000   # (PX)

W = RESOLUTION_W * PIXEL_SIZE  # (mm)
H = RESOLUTION_H * PIXEL_SIZE  # (mm)
######################################



def enu_to_ned_ned_to_enu():
    transformation_matrix = np.array([
        [0., 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, -1, 0],
		[0, 0, 0, 1]
    ])
    return transformation_matrix


def _3D_rotation_matrix(roll, pitch, yaw, order='ypr', inverso=False):
	# Return the rotation matrix in 3D coordinate systems
	yaw = np.array([
		[cos(yaw), sin(yaw), 0, 0],
		[-sin(yaw), cos(yaw), 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])
	pitch = np.array([
		[cos(pitch), 0, -sin(pitch), 0],
		[0, 1, 0, 0],
		[sin(pitch), 0, cos(pitch), 0],
		[0, 0, 0, 1]
	])
	roll = np.array([
		[1, 0, 0, 0],
		[0, cos(roll), sin(roll), 0],
		[0, -sin(roll), cos(roll), 0],
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


def ins_to_wcs():
    # Note -> WCS is ENU
        
    # 1) yaw, pitch, roll rotation
    t1 = _3D_rotation_matrix(ROLL, PITCH, YAW)
    
    # 2) NED to ENU
    t2 = enu_to_ned_ned_to_enu()
    
    return t2 @ t1

def wcs_to_ins():
    # 1) ENU to NED
    t1 = enu_to_ned_ned_to_enu()

    # 2) yaw, pitch, roll rotation
    t2 = _3D_rotation_matrix(ROLL, PITCH, YAW, inverso=True)

    return t2 @ t1



# They are the same
def gimbal_to_ins(traslation_vector):
    # 1) Translation + 2) ENU to NED
    transformation_matrix = enu_to_ned_ned_to_enu()
    transformation_matrix[:-1,-1] = traslation_vector

    return transformation_matrix

def ins_to_gimbal(traslation_vector):
    # 1) NED to ENU + 2) - translation
    transformation_matrix = enu_to_ned_ned_to_enu()
    transformation_matrix[:-1,-1] = traslation_vector

    return transformation_matrix


def camera_to_gimbal():
	# Frontal to nadiral
	# Rotation -90º axis X
	t = np.array([
		[1, 0, 0, 0],
		[0, 0, -1, 0],
		[0, 1, 0, 0],
		[0, 0, 0, 1]
	]) 
	return t

def gimbal_to_camera():
	# Nadiral to frontal
	# Rotation 90º axis X
	t = np.array([
		[1, 0, 0, 0],
		[0, 0, 1, 0],
		[0, -1, 0, 0],
		[0, 0, 0, 1]
	]) 
	return t


# 1st path: camera to wcs
t1 = camera_to_gimbal()
print("camera to gimbal:\n", t1)
t2 = gimbal_to_ins(TRANSLATION)
print("gimbal to ins:\n", t2)
t3 = ins_to_wcs()
print("ins to wcs:\n",t3)
# TOTAL
t = t3 @ t2 @ t1
print("full matrix:", t)
### Ejemplo de proyeccion
punto = np.array([3, 4, 2, 1])
proyeccion = t @ punto
print("Proyeccion del punto", punto, "a ", proyeccion)
print("\n\n")

# Camino 2 de wcs a camera
t1 = wcs_to_ins()
print("wcs to ins:\n", t1)
t2 = ins_to_gimbal(TRANSLATION)
print("ins to gimbal:\n", t2)
t3 = gimbal_to_camera()
print("gimbal to camera:\n",t3)
# TOTAL
t = t3 @ t2 @ t1
print("full matrix:", t)
### Ejemplo de proyeccion
punto = np.array([1.57246148, -3.31502868, -3.68787877, 1.])
proyeccion = t @ punto
print("Proyeccion del punto", punto, "a ", proyeccion)



# CAMERA CS (3D x,y,z) -> IMAGE CS (2D u,v)
def camera_model(X,Y,Z):
    
    x = X/Z
    y = Y/Z
    r = np.sqrt(x**2 + y**2)
    
    x_prima = x * (1 + K1*r**2 + K2*r**4 + K3*r**6 + K4*r**8 + 
        (P1*(r**2+2*x**2) + 2*P2*x*y))
    y_prima = y * (1 + K1*r**2 + K2*r**4 + K3*r**6 + K4*r**8 + 
        (P2*(r**2+2*y**2) + 2*P1*x*y))
    
    u = W * 0.5 + CX + x_prima*F + x_prima*B1 + y_prima*B2
    v = H * 0.5 + CY + y_prima*F

    return u,v

# IMAGE -> CAMERA
def image_to_camera(u,v, threshold):
    """
    De momento hecho para 1 solo pixel
    """
    
    # Obtain estimation pixels
    y_prima = (v - 0.5*H - CY) / F  
    x_prima = (u - 0.5*W - CX - y_prima*B2) / (F + B1) 

    # Initialize estimated undistorted coordinates and error
    x = x_prima
    y = y_prima
    error = np.Inf

    while(error > threshold):
        pdb.set_trace()
        r = np.sqrt(x**2 + y**2)

        # Radial distorsion
        increment_radial = K1*r**3 + K2*r**5 + K3*r**7 + K4*r**9 
        increment_x_rad = x * (increment_radial / r)
        increment_y_rad = y * (increment_radial / r)
        
        # Tangential distorsion
        increment_x_tan = B1 * (r**2 + 2*x) + 2*B2*x_prima*y_prima
        increment_y_tan = B2 * (r**2 + 2*y) + 2*B1*x_prima*y_prima
        increment_x_tan_aff = CX*x_prima + CY*y_prima

        pdb.set_trace()
        # Estimate new undistorted pixel coordinates
        x = x + increment_x_rad + increment_x_tan + increment_x_tan_aff
        y = y + increment_y_rad + increment_y_tan

        # Calculate radial and tangential distortions of (x,y)
        u_prima, v_prima = camera_model(x,y,1)
        error = np.sqrt((u - u_prima)**2 + (v - v_prima)**2)

image_to_camera(2,3,1)


