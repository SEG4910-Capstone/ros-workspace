import cv2
import numpy as np

# Create a blank white map
width, height = 620, 733  # Pixels
map_image = np.ones((height, width), dtype=np.uint8) * 255  # White background

# Draw lines
line_thickness = 1  # In pixels


# Top wall boundary - Move 4 right 4m
pt1 = (20, 20)
pt2 = (20 * 4 + 20, 20)
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Top-Right wall boundary - Move down 5.5m
pt1 = pt2
pt2 = tuple(np.add(pt1, (0, int(20 * 5.5))))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Top Vehicle Start boundary - Move right 3m
pt1 = pt2
pt2 = tuple(np.add(pt1, (int(20 * 3), 0)))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Right Vehicle Start boundary - Move down 5m
pt1 = pt2
pt2 = tuple(np.add(pt1, (0, int(20 * 5))))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Bottom Vehicle Start boundary - Move left 3m
pt1 = pt2
pt2 = tuple(np.add(pt1, (-int(20 * 3), 0)))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Bottom-Right wall boundary - Move down 5.5m
pt1 = pt2
pt2 = tuple(np.add(pt1, (0, int(20 * 5.5))))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Bottom wall boundary - Move left 4m
pt1 = pt2
pt2 = tuple(np.add(pt1, (-int(20 * 4), 0)))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Left wall boundary - Move up 16m
pt1 = pt2
pt2 = tuple(np.add(pt1, (0, -int(20 * 16))))
cv2.line(map_image, pt1, pt2, 0, thickness=line_thickness)

# Save the map as PGM
cv2.imwrite("my_map.pgm", map_image)