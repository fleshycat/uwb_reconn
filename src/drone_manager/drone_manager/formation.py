import numpy as np

class FormationForce:
    """
    Computes formation-maintaining forces using:
      - Laplacian-shape energy (complete graph)
      - scale penalty
      - distance penalties for all point pairs (edges + diagonals)
    """
    def __init__(self, desired_positions,k_scale=0.2, k_pair=1.0, k_shape=1.0, k_z=5.0):
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
        self.prev_total_error = None
        self.shape_err = None 
        self.pair_err = None 
        self.scale_err = None 
        self.z_error = None


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
        return np.nan_to_num(grad)

    def is_converged(self, points, tol=1e-1):
        shape_err_change, pair_err_change, scale_err_change, z_err_change = self.get_err_change(points)
        if shape_err_change is None or pair_err_change is None or scale_err_change is None or z_err_change is None:
            return False
        if shape_err_change < tol and pair_err_change < tol and scale_err_change < tol and z_err_change < tol:
            return True
        else:
            return False

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
        z_error = np.linalg.norm(X[:, 2] - self.Z_des)
        
        return shape_err, pair_err, scale_err, z_error
    
    def get_err_change(self, points):
        shape_err, pair_err, scale_err, z_error = self.get_error(points)
        
        if self.shape_err is not None:
            shape_err_change = abs(shape_err - self.shape_err)
        else:
            shape_err_change = None

        if self.pair_err is not None:
            pair_err_change = abs(pair_err - self.pair_err)
        else:
            pair_err_change = None

        if self.scale_err is not None:
            scale_err_change = abs(scale_err - self.scale_err)
        else:
            scale_err_change = None

        if self.z_error is not None:
            z_error_change = abs(z_error - self.z_error)
        else:
            z_error_change = None

        self.shape_err = shape_err
        self.pair_err = pair_err
        self.scale_err = scale_err
        self.z_error = z_error
        return (shape_err_change, pair_err_change, scale_err_change, z_error_change)