import numpy as np
import math

def normalizeArray(list, highestList, lowestList):
    newList = np.zeros(shape=(14,1))
    for i in range(0, list.shape[0]):
        #print (list[i])
        #print (highestList[i])
        newList[i] = ((list[i] - lowestList[i])/(highestList[i] - lowestList[i]))*(0.9-(0.1)) + 0.1 
                    
    return newList
    
def desnormalizeArray(list, highestList, lowestList):
    newList = np.zeros(shape=(7,1))
    for i in range(0, list.shape[0]):
        newList[i] = ((list[i]-0.1)*(highestList[i] - lowestList[i])/((0.9-(0.1))) + lowestList[i])
                
    return newList

# def quaternion_to_euler_angle(x, y, z, w):
# 	ysqr = y * y
	
# 	t0 = +2.0 * (w * x + y * z)
# 	t1 = +1.0 - 2.0 * (x * x + ysqr)
# 	X = math.degrees(math.atan2(t0, t1))
	
# 	t2 = +2.0 * (w * y - z * x)
# 	t2 = +1.0 if t2 > +1.0 else t2
# 	t2 = -1.0 if t2 < -1.0 else t2
# 	Y = math.degrees(math.asin(t2))
	
# 	t3 = +2.0 * (w * z + x * y)
# 	t4 = +1.0 - 2.0 * (ysqr + z * z)
# 	Z = math.degrees(math.atan2(t3, t4))
	
# 	return X, Y, Z

def angle_limit(a):
    return math.atan2(math.sin(a),math.cos(a))

def quaternion_to_euler_angle(x, y, z, w):
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.atan2(t0, t1)
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.atan2(t3, t4)
	
	return angle_limit(X), angle_limit(Y), angle_limit(Z)
