import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer

# import datetime
# starttime = datetime.datetime.now()

# lib = ctypes.CDLL('./Normalize_height.so')
# lib.C_compute_plane.restype = ndpointer(dtype=ctypes.c_float, shape=(4,))

# lib.C_compute_plane.argtypes = (ctypes.c_double,ctypes.c_double,ctypes.c_double \
#                                 ,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double)
# res = lib.C_compute_plane(1,1,1,2,2,2,3,3,3)
# print(res)


# endtime = datetime.datetime.now()
# print (endtime - starttime)
rast = np.array([[1,2,3,0],
                 [1,0,0,1],
                 [6,7,8,8]])
# rast = np.array([1,2,3,0,1,0,0,1,6,7,8,8])

# To C++
lib = ctypes.cdll.LoadLibrary("./Count_empty_pixel.so")
lib.count_surrounding_zeros.argtypes = (ctypes.c_char_p,ctypes.c_int,ctypes.c_int)
dataptr = rast.ctypes.data_as(ctypes.c_char_p)
rows,columns = rast.shape[0],rast.shape[1] 
local_maximum = lib.count_surrounding_zeros(dataptr,rows,columns) # c++ version
print(local_maximum)