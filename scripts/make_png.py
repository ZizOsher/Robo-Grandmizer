import numpy as np
import matplotlib.pyplot as plt

# Load the data from the text file into a numpy array.
data = np.loadtxt('../binary_image.txt')

# Flip the data in the up/down direction.
data = np.flipud(data)

# Create a binary colormap for the image.
cmap = plt.cm.colors.ListedColormap(['black', 'white'])

# Create the image using matplotlib, note origin is set to 'lower' so the (0,0) index is at the bottom-left corner
plt.imshow(data, cmap=cmap, origin='lower')

# Remove axes for a cleaner image.
plt.axis('off')

# Save the image.
plt.savefig('../floor.png')

