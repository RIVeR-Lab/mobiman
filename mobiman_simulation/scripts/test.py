import numpy as np

# Given data
d0 = 5
d1_min = 0.5
r_max = 10
d_th = 2

# Calculate d1_max
d1_max = 9.5

# Calculate the slope of the tangent line at d1_min
slope = -r_max * (d0 / (d_th * d0))  # derivative of the function with respect to d1

# Calculate the intercept of the tangent line
intercept = -slope * d1_min + (r_max * (d0 - d1_min) / d0) * ((d1_min / d_th) + 1)

# Define the convex lower bound function
def convex_lower_bound(d1):
    return slope * d1 + intercept

# Test the function
d1_test = np.linspace(d1_min, d1_max, 100)
lower_bound_values = convex_lower_bound(d1_test)

# Plot the original function and its convex lower bound
import matplotlib.pyplot as plt

r_values = -r_max * (d0 - d1_test) / d0 * ((d1_test / d_th) + 1)

plt.plot(d1_test, r_values, label='Original Function')
plt.plot(d1_test, lower_bound_values, label='Convex Lower Bound', linestyle='--')
plt.xlabel('d1')
plt.ylabel('r')
plt.legend()
plt.show()
