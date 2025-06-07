import numpy as np
from collections import deque

class FormationForce:
    """
    Computes formation-maintaining forces using:
      - Laplacian-shape energy (complete graph)
      - scale penalty
      - distance penalties for all point pairs (edges + diagonals)
    """
    def __init__(self, desired_positions, k_scale=0.2, k_pair=1.0, k_shape=1.0, k_z=5.0, tol=1e-3):
        self.X_des = np.array(desired_positions, dtype=float)
        self.n = self.X_des.shape[0]
        # weights
        self.k_scale = k_scale
        self.k_pair  = k_pair
        self.k_shape = k_shape
        self.k_z = k_z
        # build complete-graph Laplacian
        L = -np.ones((self.n, self.n))
        np.fill_diagonal(L, self.n - 1)
        self.L = L
        self.Z_des = self.X_des[:, 2]
        
        # desired raw signature and trace
        S0_des = self.X_des.T @ L @ self.X_des # shape (3,3)
        tr_des = np.trace(S0_des)
        self.S_des_norm = S0_des / tr_des
        self.tr_des_raw = tr_des
        # all point-pair edges (including diagonals)
        self.pair_edges = [(i, j)
                           for i in range(self.n)
                           for j in range(i+1, self.n)]
        # desired pairwise distances
        self.pair_dist = [np.linalg.norm(self.X_des[i] - self.X_des[j])
                          for i, j in self.pair_edges]
        
        self.tol = tol
        self.grad = []
    
    def set_desired_formation(self, desired_positions):
        self.X_des = np.array(desired_positions, dtype=float)
        self.n = self.X_des.shape[0]
        # update Laplacian
        L = -np.ones((self.n, self.n))
        np.fill_diagonal(L, self.n - 1)
        self.L = L
        self.Z_des = self.X_des[:, 2]
        
        # update desired raw signature and trace
        S0_des = self.X_des.T @ L @ self.X_des
        tr_des = np.trace(S0_des)
        self.S_des_norm = S0_des / tr_des
        self.tr_des_raw = tr_des
        # update pairwise edges and distances
        self.pair_edges = [(i, j)
                           for i in range(self.n)
                           for j in range(i+1, self.n)]
        self.pair_dist = [np.linalg.norm(self.X_des[i] - self.X_des[j])
                          for i, j in self.pair_edges]

    def compute(self, points):
        X = np.array(points)   # shape (n,3)
        # raw and normalized shape signature
        S0 = X.T @ self.L @ X     # shape (3,3)
        trS = np.trace(S0)
        S_norm = (S0 / trS) if trS > 1e-8 else np.zeros_like(S0)
        
        # shape gradient
        grad_shape = 2 * (self.L @ X) @ (S_norm - self.S_des_norm)
        # scale penalty (trace difference)
        grad_scale = 4 * (trS - self.tr_des_raw) * (self.L @ X) / (trS + 1e-8)
        
        # pairwise distance penalties (edges + diagonals)
        grad_pair = np.zeros_like(X)
        for (i, j), d_des in zip(self.pair_edges, self.pair_dist):
            diff = X[i] - X[j]
            dist = np.linalg.norm(diff)
            if dist > 1e-6:
                err = dist - d_des
                # gradient of (dist - d_des)^2 w.r.t. X positions
                force = 2 * err * (diff / dist)
                grad_pair[i] += force
                grad_pair[j] -= force
        grad_z = np.zeros_like(X)
        z_err = X[:, 2] - self.Z_des
        grad_z[:, 2] = 2.0 * self.k_z * z_err

        # total gradient
        grad = (self.k_shape * grad_shape
                + self.k_scale * grad_scale
                + self.k_pair  * grad_pair
                + grad_z) 
        self.grad = grad
        return np.nan_to_num(grad)

    def get_error(self, points):
        X = np.array(points)
        S0 = X.T @ self.L @ X
        trS = np.trace(S0)
        S_norm = (S0 / trS) if trS > 1e-8 else np.zeros_like(S0)
        
        shape_err = np.linalg.norm(S_norm - self.S_des_norm)
        pair_err = sum(
            abs(np.linalg.norm(X[i] - X[j]) - d_des)
            for (i, j), d_des in zip(self.pair_edges, self.pair_dist)
        )
        scale_err = abs(trS - self.tr_des_raw)
        # z_error = np.linalg.norm(X[:, 2] - self.Z_des)
        
        # return shape_err, pair_err, scale_err, z_error
        return shape_err, pair_err, scale_err