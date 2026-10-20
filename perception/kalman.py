"""Scalar Kalman filters used to fuse repeated noisy measurements."""


class Kalman1D:
    """Constant-state Kalman filter: x_k = x_{k-1} + w,  z_k = x_k + v."""

    def __init__(self, q=1e-7, r=1e-5):
        self.q, self.r = q, r
        self.reset()

    def reset(self, x0=None, p0=1.0):
        self.x = x0
        self.p = p0
        self.n = 0

    def predict(self, u=0.0):
        """Optional control input u (e.g. odometry displacement)."""
        if self.x is not None:
            self.x += u
            self.p += self.q

    def update(self, z, r=None):
        if z is None:
            return self.x
        r = self.r if r is None else r
        if self.x is None:
            self.x, self.p = z, r
        else:
            self.p += self.q
            k = self.p / (self.p + r)
            self.x += k * (z - self.x)
            self.p *= (1 - k)
        self.n += 1
        return self.x

    @property
    def std(self):
        return self.p ** 0.5
