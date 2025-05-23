import numpy as np

class TargetForce:
    def __init__(self, target_position, k_mission=1.0):
        # target_position: (x,y,z)
        self.X_tar = np.array(target_position, dtype=float)
        self.k = k_mission

    def compute(self, current_pos, target):
        diff = np.array(current_pos) - np.array(target)         # shape (3,)
        
        # gradient ∇E = 2 * diff * k
        grad = 2.0 * diff * self.k             # shape (3,)
        
        # guard against NaN / inf
        return np.nan_to_num(grad)