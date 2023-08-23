from PIL import Image
import numpy as np
import os

# image_path = 'bitmaps/floor-3.png'
image_path = '../csfloor.png'

# Open the image file
with Image.open(image_path) as img:
    # Convert image to grayscale
    grayscale_img = img.convert('L')

# Convert grayscale image to numpy array
image_array = np.array(grayscale_img)

# Apply threshold
binary_image_array = np.where(image_array < 128, 1, 0)

# Calculate and print the percentage of black and white pixels
num_pixels = binary_image_array.size
num_white_pixels = np.sum(binary_image_array == 0)
num_black_pixels = np.sum(binary_image_array == 1)

print("Percentage of white pixels: ", (num_white_pixels / num_pixels) * 100)
print("Percentage of black pixels: ", (num_black_pixels / num_pixels) * 100)

# Get the dimensions of the image
num_rows, num_columns = binary_image_array.shape
print("Number of rows: ", num_rows)
print("Number of columns: ", num_columns)


# Save the binary image array to a text file
with open('../binary_image.txt', 'w') as f:
    for row in binary_image_array:
        row_string = ' '.join(str(pixel) for pixel in row)
        f.write(row_string + os.linesep)

