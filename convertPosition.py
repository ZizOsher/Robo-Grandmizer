def world_to_pixel(x, y, image_width=287, image_height=369, world_width=43.780, world_height=56.040):
    """Convert world coordinates to pixel coordinates."""
    px = int(round((x/world_width + 0.5) * image_width))
    py = int(round((0.5 - y/world_height) * image_height))
    
    # Ensure the calculated pixel values are within image boundaries
    px = max(0, min(px, image_width-1))
    py = max(0, min(py, image_height-1))
    
    return px, py

def pixel_to_world(px, py, image_width=287, image_height=369, world_width=43.780, world_height=56.040):
    """Convert pixel coordinates to world coordinates."""
    x = round((px/image_width - 0.5) * world_width, 6)  # rounding to 6 decimal places
    y = round((0.5 - py/image_height) * world_height, 6)  # rounding to 6 decimal places
    
    return x, y

# Test
x, y = -8, -14
px, py = world_to_pixel(x, y)
x_new, y_new = pixel_to_world(px, py)
print(f"World: ({x}, {y}) -> Pixel: ({px}, {py}) -> World: ({x_new}, {y_new})")


# Test
x, y = 0, 0
px, py = pixel_to_world(x, y)
x_new, y_new = world_to_pixel(px, py)
print(f"Pixel: ({x}, {y}) -> World: ({px}, {py}) -> Pixel: ({x_new}, {y_new})")