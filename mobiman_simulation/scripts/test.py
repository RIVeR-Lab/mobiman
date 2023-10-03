import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

def scaled_gamma_pdf_single(x, shape, scale):
    gamma_pdf = stats.gamma.pdf(x, shape, scale=scale)
    min_pdf = np.min(gamma_pdf)
    max_pdf = np.max(gamma_pdf)
    normalized_pdf = -1 + 2 * ((gamma_pdf - min_pdf) / (max_pdf - min_pdf))
    return normalized_pdf

# Define shape and scale parameters
shape = 2
scale = 1

# Choose an x value
x_value = 5

# Calculate normalized PDF for the chosen x value
normalized_pdf_value = scaled_gamma_pdf_single(x_value, shape, scale)

# Print the result
print(f"For x = {x_value}, normalized PDF = {normalized_pdf_value}")

# Plotting the scaled gamma distribution
x_values = np.linspace(0, 10, 1000)
normalized_pdf = [scaled_gamma_pdf_single(x, shape, scale) for x in x_values]

plt.plot(x_values, normalized_pdf)
plt.xlabel('x')
plt.ylabel('Normalized PDF')
plt.title(f'Scaled Gamma Distribution (shape={shape}, scale={scale})')
plt.show()
