import numpy as np
import json

class State:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
    
    def __repr__(self):
        return repr(self.__dict__)

def normalize_angle(angle):
    return np.mod(angle + np.pi, 2*np.pi) - np.pi

def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def unit(v): return v / np.linalg.norm(v)

class NumpyEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        else:
            super().default(o)