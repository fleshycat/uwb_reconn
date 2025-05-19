import numpy as np

class TargetForce:
    def __init__(self, target_position, k_mission=1.0):
        self.X_tar = np.array(target_position, dtype=float)
        self.k = k_mission

    def compute(self, point, target):
        diff = point - target         # shape (2,)
        
        # gradient ∇E = 2 * diff * k
        grad = 2.0 * diff * self.k             # shape (2,)
        
        # guard against NaN / inf
        return np.nan_to_num(grad)