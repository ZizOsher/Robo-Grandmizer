def world_to_pixel(x, y, image_width=287, image_height=369, world_width=43.780, world_height=56.040):
    """Convert world coordinates to pixel coordinates."""
    px = int(round((x/world_width + 0.5) * image_width))
    py = int(round((0.5 - y/world_height) * image_height))
    
    px = max(0, min(px, image_width-1))
    py = max(0, min(py, image_height-1))
    
    return px, py

def pixel_to_world(px, py, image_width=287, image_height=369, world_width=43.780, world_height=56.040):
    """Convert pixel coordinates to world coordinates."""
    x = round((px/image_width - 0.5) * world_width, 6)
    y = round((0.5 - py/image_height) * world_height, 6)
    
    return x, y

def world_distance_to_pixel_distance(distance, image_width=287, world_width=43.780):
    """Convert a distance in the simulation world to a distance in pixels."""
    return int(round(distance * image_width / world_width))

def pixel_distance_to_world_distance(pixel_distance, image_width=287, world_width=43.780):
    """Convert a distance in pixels to a distance in the simulation world."""
    return round(pixel_distance * world_width / image_width, 6)


# Test 1
x, y = -8, -14
px, py = world_to_pixel(x, y)
x_new, y_new = pixel_to_world(px, py)
print(f"World: ({x}, {y}) -> Pixel: ({px}, {py}) -> World: ({x_new}, {y_new})")


# Test 2
x, y = 0, 0
px, py = pixel_to_world(x, y)
x_new, y_new = world_to_pixel(px, py)
print(f"Pixel: ({x}, {y}) -> World: ({px}, {py}) -> Pixel: ({x_new}, {y_new})")

# Distance Test
dist_meters = 10
dist_pixels = world_distance_to_pixel_distance(dist_meters)
dist_meters_new = pixel_distance_to_world_distance(dist_pixels)

print(f"World distance: {dist_meters} meters -> Pixel distance: {dist_pixels} pixels -> World distance: {dist_meters_new} meters")
