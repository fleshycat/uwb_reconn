import numpy as np

class RepulsionForce:
    """
    Computes pairwise repulsion to avoid agent collisions.
    F_rep[i] = sum_{j != i} c_rep * (x_i - x_j) / dist^p  for dist < cutoff
    """
    def __init__(self, n_agents, c_rep=5.0, sigma=1.0, cutoff=3.0):
        self.n = n_agents
        self.c_rep = c_rep
        self.cutoff = cutoff
        self.sigma = sigma

    def compute(self, points):
        # points: list of objects with .pos = (x,y,z)
        X = np.array(points)  # shape (n,3)
        grad_rep = np.zeros_like(X)
        for i in range(self.n):
            for j in range(self.n):
                if i == j: continue
                diff = X[i] - X[j]          # shape (3,)
                dist = np.linalg.norm(diff)
                if 1e-6 < dist < self.cutoff:
                    weight = np.exp(-0.5 * (dist / self.sigma)**2)
                    grad_rep[i] += self.c_rep * (diff / dist) * weight
        return grad_rep