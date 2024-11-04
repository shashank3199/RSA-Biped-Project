import sys
from stl import mesh
import numpy as np

def find_geometric_center(stl_file):
    # Load STL file
    stl_mesh = mesh.Mesh.from_file(stl_file)

    # Extract all vertices from the mesh
    vertices = np.concatenate([stl_mesh.v0, stl_mesh.v1, stl_mesh.v2])

    # Find minimum and maximum coordinates along each axis
    min_coords = np.min(vertices, axis=0)
    max_coords = np.max(vertices, axis=0)

    # Calculate geometric center (midpoint between min and max coordinates)
    geometric_center = (min_coords + max_coords) / 2
    geometric_center /= 1000  # Convert from millimeters to meters
    geometric_center *= -1  # Invert direction

    return f"{geometric_center[0]:.3f} {geometric_center[1]:.3f} {geometric_center[2]:.3f}"

stl_file_path = sys.argv[1]
center_coordinates = find_geometric_center(stl_file_path)
print(center_coordinates)