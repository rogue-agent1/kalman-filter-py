#!/usr/bin/env python3
"""Kalman filter — 1D and matrix implementations."""

class KalmanFilter1D:
    def __init__(self, x0, p0, q, r):
        self.x=x0; self.p=p0; self.q=q; self.r=r
    def predict(self, u=0):
        self.x += u; self.p += self.q
    def update(self, z):
        k = self.p/(self.p+self.r)
        self.x += k*(z-self.x); self.p *= (1-k)
        return self.x

def kalman_1d(measurements, x0=0, p0=1, q=0.01, r=1):
    kf = KalmanFilter1D(x0, p0, q, r); estimates = []
    for z in measurements:
        kf.predict(); estimates.append(kf.update(z))
    return estimates

def main():
    import random; random.seed(42)
    true_val = 5.0
    measurements = [true_val + random.gauss(0, 1) for _ in range(20)]
    estimates = kalman_1d(measurements, x0=0, p0=100, q=0.01, r=1)
    print(f"True: {true_val}, Final estimate: {estimates[-1]:.4f}")

if __name__ == "__main__": main()
