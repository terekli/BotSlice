def find_intersection(triangle, slice_z):
    
    import numpy as np
    
    # Check how many vertices in directly on the slice plane
    count_equal_to_slice = np.sum(triangle[:, 2] == slice_z)
    
    # Only one vertex on the plane
    if count_equal_to_slice == 1:
        ind = triangle[:,2] == slice_z
        edge = triangle[ind,0:2]
        return edge
    
    # Two vertices on the plane: one edge is on the plane
    if count_equal_to_slice == 2:
        ind = triangle[:,2] == slice_z 
        edge = triangle[ind,0:2]
        return edge
    
    # Three vertices on the plane: triangle is flat on the plane
    if count_equal_to_slice == 3:
        # Return all three vertices
        edge = triangle[:,0:2]
        return edge
    
    # Other cases
    edge = []
    for i in range(3): # Each triangle has 3 edges
        # Get the current and next vertex to form an edge
        p1, p2 = triangle[i], triangle[(i + 1) % 3]
        
        # (x,y) coordinate where the triangle intersect the slice plane
        # only add to edge if a value is obtained
        temp = interpolation(p1, p2, slice_z)
        if ~np.isnan(temp).any():
            edge.append(temp)

    # convert output from list into ndarray
    edge = np.array(edge)
    
    return edge        

def interpolation(p1, p2, slice_z):
    
    # Function: interpolate between two points (p1, p2) in 3D space, and find the intersection
    # at a certain height (slice_z)

    # Input: p1 and p2: (x,y,z) coordinate, represent the vertices of a triangle
    # slice_z: height of slice plane
    
    import numpy as np
            
    # Compute the vector between point 1 and 2 on the line in 3D
    vector = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    
    # Relative z = height between slice z and the z of the lower point
    rel_z = slice_z - p1[2]
    
    # Check if the line segment is parallel to the XY plane
    if vector[2] == 0:
        # The entire segment is on the slice plane, then the two other edges of the triangle
        # must also intersect the slice plane, so we can use the intersection point of the 
        # other two edges
        return([[np.nan, np.nan]])

    # Calculate parametric length along the vector
    a = rel_z / vector[2]
    
    # Compute new X and Y points at that parametric length
    points = [a * vector[0] + p1[0], a * vector[1] + p1[1]]

    return points
