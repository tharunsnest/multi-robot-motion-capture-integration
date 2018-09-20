from source import Arr_s
import numpy as np
Arr_S =np.array(Arr_s)
from multiprocessing import Pool
from moveaway_h import funct #due to OS constraints, implemented in a separate script
from moveaway_h import publishing






if __name__ == '__main__':
    pool = Pool(len(Arr_S))
    Arr_D = pool.map(funct,Arr_S)
    Arr_D_h = np.column_stack((Arr_D,np.arange(1,len(Arr_D)+1)))
    pool.map(publishing,Arr_D_h)

