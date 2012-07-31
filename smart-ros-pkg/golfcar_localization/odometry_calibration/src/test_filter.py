import roslib; roslib.load_manifest('odometry_calibration')
import matplotlib.pyplot as plt
import numpy as np
from odometry_calibration import KalmanFilter, FilterState, WeightedAverageFilter

# the measurement noise
sigma_r = .1

# the true value
X = 1.0

# the measurements
z = X + np.random.randn(50) * sigma_r

# the prior
prior = FilterState(X + np.random.randn() * sigma_r, sigma_r)

# the filters
f_kf = KalmanFilter(prior, q=0.0001)
f_wa = WeightedAverageFilter(prior)

# the results
x_kf = [prior]
x_wa = [prior]

for zi in z[1:]:
    for x,f in [(x_kf, f_kf), (x_wa, f_wa)]:
        f.update(zi, sigma_r)
        x.append(f.get_state())

plt.subplot(211)
plt.plot([x.x for x in x_kf], 'b', label="estimation kalman")
plt.plot([x.x for x in x_wa], 'g', label="estimation weighted average")
plt.plot(z, 'r.', label="measurement")
plt.plot(np.zeros_like(z)+X, 'k-', label="true value")
plt.legend()
plt.subplot(212)
plt.plot([x.p for x in x_kf], 'b', label="variance kalman")
plt.plot([x.p for x in x_wa], 'g', label="variance weighted average")
plt.plot(np.zeros_like(z)+sigma_r, 'k-', label="measurement variance")
plt.legend()
plt.show()