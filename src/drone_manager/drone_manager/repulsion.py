import numpy as np

class RepulsionForce:
    """
    Computes pairwise repulsion to avoid agent collisions.
    F_rep[i] = sum_{j != i} c_rep * (x_i - x_j) / dist^p  for dist < cutoff
    """
    def __init__(self, n_agents, c_rep=5, cutoff=3, p=2):
        self.n = n_agents
        self.c_rep = c_rep
        self.cutoff = cutoff
        self.p = p

    def compute(self, points):
        X = np.vstack([p.pos for p in points])
        grad_rep = np.zeros_like(X)
        for i in range(self.n):
            for j in range(self.n):
                if i == j: continue
                diff = X[i] - X[j]
                dist = np.linalg.norm(diff)
                if 1e-6 < dist < self.cutoff:
                    grad_rep[i] += self.c_rep * diff / (dist**self.p)
        return grad_rep