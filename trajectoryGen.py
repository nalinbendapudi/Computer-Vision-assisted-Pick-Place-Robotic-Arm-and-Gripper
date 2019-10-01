import numpy as np

def cubicInterpolation (initialAngle, finalAngle, initialVelocity, finalVelocity, totalTime, numberOfIneterpolations):
	# Return a 2 X (numberOfIneterpolations) list of angles and velocities
	# Cubic interpolation on angle assuming given initial and final angles and velocities
	
	timeSteps  = numberOfIneterpolations+1
	
	t_o = 0
	t_f = totalTime
	q_o = initialAngle
	q_f = finalAngle
	v_o = initialVelocity
	v_f = finalVelocity
	
	M = np.array([[1,t_o,t_o**2,t_o**3], [0,1,2*t_o,3*t_o**2], [1,t_f,t_f**2,t_f**3], [0,1,2*t_f,3*t_f**2]])
	b = np.array([q_o,v_o,q_f,v_f])
	a = np.linalg.solve(M,b)
	
	angles = []
	velocities = []
	
	for i in range(1,timeSteps):
		t = t_o + i*(t_f-t_o)/timeSteps
		q = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
		v = a[1] + a[2]*2*t + a[3]*3*t**2
		angles.append(q)
		velocities.append(v)
	
	return velocities, angles
		
		
	
	
	
	
