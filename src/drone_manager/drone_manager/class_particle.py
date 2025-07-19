#algo_2d

import numpy as np

class ParticleFilter:

    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = None
        self.weights   = None
        self.process_noise_std = 0.3
        self.region_radius = 20.0
        self.step_counter = 0
    
    def set_num_particles(self, num_particles):
        self.num_particles = num_particles
        self.particles = None
        self.weights   = None

    def initialize(self, sensor_position):
        cx, cy = sensor_position
        r = self.region_radius
        
        angles = np.random.uniform(0, 2*np.pi,    size=self.num_particles)
        radii  = r * np.sqrt(np.random.uniform(size=self.num_particles))
        xs = cx + radii * np.cos(angles)
        ys = cy + radii * np.sin(angles)
        
        self.particles = np.column_stack((xs, ys))
        self.weights   = np.ones(self.num_particles) / self.num_particles
    
    def inject_shared_target(self, external_estimates, n_shared=50, sigma=0.5):
        if self.particles is None:
            return
        
        shared_particles = []
        m = len(external_estimates)
        if m == 0:
            return
        k = int(np.ceil(n_shared / m))
        for pos in external_estimates:
            cx, cy = pos
            xs = np.random.normal(cx, sigma, size=k)
            ys = np.random.normal(cy, sigma, size=k)
            shared_particles.append(np.column_stack((xs, ys)))
        shared = np.vstack(shared_particles)[:n_shared]

        all_particles = np.vstack((self.particles, shared))
        all_weights   = np.hstack((self.weights, np.full(n_shared, 1.0/n_shared)))
        
        idx = np.argsort(all_weights)[-self.num_particles:]
        self.particles = all_particles[idx]
        self.weights   = all_weights[idx]
        self.weights /= np.sum(self.weights)
        
    def inject_from_metrics(self, metrics_list, n_shared=100):
        if self.particles is None or len(metrics_list) == 0:
            return

        k = int(np.ceil(n_shared / len(metrics_list)))
        shared_parts = []

        for sensor_pos, avg_r, b_min_deg, b_max_deg, r_std in metrics_list:
            # sample angles in [bearing_min, bearing_max]
            angles = np.random.uniform(
                np.radians(b_min_deg),
                np.radians(b_max_deg),
                size=k
            )
            # sample radii around avg_r with std=r_std
            radii = np.random.normal(loc=avg_r, scale=r_std, size=k)
            # polar → cartesian
            xs = sensor_pos[0] + radii * np.cos(angles)
            ys = sensor_pos[1] + radii * np.sin(angles)
            shared_parts.append(np.column_stack((xs, ys)))

        # stack and trim to exactly n_shared
        shared = np.vstack(shared_parts)[:n_shared]

        # merge with existing particles
        all_particles = np.vstack((self.particles, shared))

        # keep old weights, give new ones a small equal weight
        w_old = self.weights
        w_new = np.full(n_shared, 1.0 / (n_shared * 10))
        all_weights = np.hstack((w_old, w_new))
        all_weights /= np.sum(all_weights)

        # resample back down to original count
        choose_idx = np.random.choice(
            a=all_particles.shape[0],
            size=self.num_particles,
            p=all_weights
        )
        self.particles = all_particles[choose_idx]
        # reset to uniform after injection
        self.weights = np.ones(self.num_particles) / self.num_particles
          
    def predict(self):
        noise = np.random.normal(
            loc=0.0,
            scale=self.process_noise_std,
            size=self.particles.shape
        )
        self.particles += noise
        
    def update(self, sensor_positions, measurements, noise_stds):
        for pos, meas, ns in zip(sensor_positions, measurements, noise_stds):
            expected = np.linalg.norm(self.particles - pos, axis=1)
            eps = 1e-6
            std = np.maximum(ns, eps)
            likelihood = np.exp(-0.5 * ((meas - expected)/ (std)) **2 ) 
            self.weights *= likelihood
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

    def resample(self):
        indices = np.random.choice(
            self.num_particles, size=self.num_particles, p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def step(self, point_pos, measurements, noise_std, current_pos):
        if self.particles is None:
            if len(measurements) > 0:
                self.initialize(current_pos)
            else:
                return np.zeros((self.num_particles,2)), np.ones(self.num_particles)/self.num_particles
        # 1) Prediction: spread particles
        self.update(point_pos, measurements, noise_std)
        self.predict()
        self.resample()
        self.step_counter += 1
        # if self.step_counter >= 80:
        #     self.initialize(current_pos)
        #     self.step_counter = 0
        
        
        # center = self.estimate()
        center = current_pos 
        
        dists = np.linalg.norm(self.particles - center, axis=1)
        mask_inside = (dists <= self.region_radius)
        num_inside = mask_inside.sum()
        num_new = self.num_particles - num_inside

        if num_new > 0:
            kept = self.particles[mask_inside]
            angles = np.random.uniform(0, 2*np.pi, size=num_new)
            radii  = self.region_radius * np.sqrt(np.random.uniform(size=num_new))
            xs = center[0] + radii * np.cos(angles)
            ys = center[1] + radii * np.sin(angles)
            spawned = np.column_stack((xs, ys))
            self.particles = np.vstack((kept, spawned))
            self.weights   = np.ones(self.num_particles) / self.num_particles
        
        return self.particles, self.weights

    def distribution_metrics(self, sensor_position):
        if self.particles is None or len(self.particles) == 0:
            return None  # no particles

       # 2) Relative vectors, distances and bearings
        rel = self.particles - sensor_position      # shape (N,2)
        distances = np.linalg.norm(rel, axis=1)
        avg_radius = np.mean(distances)
        radius_std = np.std(distances)

        bearings = np.degrees(np.arctan2(rel[:,1], rel[:,0]))
        bearing_min = np.min(bearings)
        bearing_max = np.max(bearings)

        return sensor_position, avg_radius, bearing_min, bearing_max, radius_std

    def estimate(self):
        if self.particles is not None:
            return np.average(self.particles, weights=self.weights, axis=0)
        else:
            return None
    
    def covariance(self):
        mu = self.estimate()                          
        diffs = self.particles - mu                
        cov = (self.weights[:,None] * diffs).T @ diffs
        return cov  # 2×2 행렬

    def effective_sample_size(self):
        return 1.0 / np.sum(self.weights**2)