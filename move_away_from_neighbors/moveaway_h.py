#import Array of source coordinates
from destination import Arr_S
import numpy as np
def funct(a):
    alpha_sample = 0.1
    v = [0,0]
    for i in range(len(Arr_S)):
        v = v + a - Arr_S[i]
        v_h = v/np.linalg.norm(v)
        v_h = v_h * alpha_sample
    return a+v_h

def publishing(a):
    

    