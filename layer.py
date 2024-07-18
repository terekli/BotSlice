########## START OF interplation ##########
def interpolation(p1, p2, slice_z):

    """
    Interpolate between two points (p1, p2) in 3D space, find the intersection at slize_z

    Input:
    p1 and p2: (x, y, z) representing the vertices of a triangle
    slize_z: height of a slice plane in mm
    """
    
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
########## END OF interplation ##########



########## START OF find_intersection ##########
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
########## START OF find_intersection ##########



########## START OF get_edge_points ##########
def get_edge_points(model, slice_z):

    """
    Calculate all the edge points of the model at a certain slice plane

    Input:
    model: from mesh.Mesh.from_file(filename)
    slize_z: height of the slice plane in mm

    Output:
    edge_points: each row represent the (x, y) of edge points at the slice plane
    solid_directions: each row represent the normal vector of each edge points
    """
    
    import numpy as np

    connected_points = []
    normal = []
    
    # Loop through each triangle
    for itriangle in range(np.shape(model.vectors)[0]):
        
        # grab the triangle
        triangle = model.vectors[itriangle,:,:]
        
        # Only try to find edge points if triangle intersect the slicing plane
        # and is above the slicing plane
        if np.min(triangle[:,2]) <= slice_z <= np.max(triangle[:,2]):

            connected_points.append(np.round(find_intersection(triangle, slice_z), 2))
            normal.append(model.normals[itriangle,:])
            
    return connected_points, normal
########## END OF get_edge_points ##########



########## START OF close_to ##########
def relative_distance(pair, destination):
    
    import numpy as np

    """
    Not currently used.
    Find which points in a pair of points is closest to the destination

    Input:
    pair: pair of points in ndarray, size (2, 2)
    destination: destination coordinate (x,y)

    Output:
    1x2 array pointer on the order of closest to destination
    0 is closer to the destination
    1 is further to the destination
    """
    
    point1 = pair[0,:]
    point2 = pair[1,:]
    
    point1_distance = np.sqrt((point1[0]-destination[0])**2 + (point1[1]-destination[1])**2)
    point2_distance = np.sqrt((point2[0]-destination[0])**2 + (point2[1]-destination[1])**2)
    
    if point1_distance < point2_distance:
        return [0,1]
    else:
        return [1,0]
########## END OF close_to ##########



########## START OF closest_pair ##########
def closest_pair(contours, connected_points):
    
    """
    Not currently used.
    Find closest points in connected_points to the last point in contour

    Input:
    contours
    connected_points

    Output:
    ind: index of closest pairs in connected_points
    """
    
    last_point = contours[-1]
    
    # Find the ind of connected_points that contain the last point in contour
    for index, pair in enumerate(connected_points):
        # Check if the point is in the entry
        if (pair == [last_point[0], last_point[1]]).all(axis=1).any():
            return index
 ########## END OF closest_pair ##########



########## START OF build_contours ##########
def build_contours(connected_points, normal_vector):

    """
    Clean up the point-pairs in connected_points and synchronize with normal_vector.
    Make generation of layer-map more efficient.

    Input:
    connected_points: a list, each index is a 2x2 ndarray representing the (x, y) of two connected points
    normal_vector: a list, same length as connected_points, each index is the normal vector of the corresponding connected points
    negative vector indicates pointing out of the model

    Output:
    contours: same structure as connected_points
    normal_vector
    """

    # Remove entries that are triangle or single point
    filtered_connected_points = []
    filtered_normal_vector = []
    for index, entry in enumerate(connected_points):
        if entry.shape != (1, 2) and entry.shape != (3, 2):
            filtered_connected_points.append(entry)
            filtered_normal_vector.append(normal_vector[index])
    
    connected_points = filtered_connected_points
    normal_vector = filtered_normal_vector

    contours = connected_points

    # # Inititae
    # contours = []

    # # Whether the contour is closed
    # closed = True

    # while len(connected_points) != 0:

    # # Process the pairs of connected points
    # while len(connected_points) != 0:
        
    #     if closed is False:
    #         # Find closest point in connected_points to last point in contours
    #         ind = closest_pair(contours, connected_points)
            
    #         # If no closest pair is found, which often occurs for complex or corrupted model
    #         if ind is None:
    #             a = 1
                
    #         temp = relative_distance(connected_points[ind], contours[-1])
            
    #         if temp is None:
    #             a = 1
                
    #         contours.append(connected_points[ind][temp[1],:])
    #         connected_points.pop(ind)
    #     else: # closed is True
    #         # Start a new contour with the first pair in connected_points
    #         closed = False
    #         contours_start = connected_points[0][0]
    #         contours.append(contours_start)
    #         contours.append(connected_points[0][1])
    #         connected_points.pop(0)
        
    #     # Check if a closed contour is formed
    #     if np.array_equal(contours_start, contours[-1]):
    #         closed = True
    #         # Each closed contour is seperated by [nan, nan]
    #         contours.append(np.array([np.nan, np.nan]))
            
    # # Remove consecutive entries that are the same
    # # This generate accurate number of closed contours
    # contours = [contours[i] for i in range(len(contours)) if i == 0 or not np.array_equal(contours[i], contours[i-1])]

    # # Organize the contours into a 3D array, where each z-direction is a closed contour
    # separator_indices = np.where(np.all(np.isnan(contours), axis=1))[0]
    # contours = np.split(contours, separator_indices+1)

    return contours, normal_vector
########## END OF build_contours ##########



########## START OF distance_point_to_line ##########
def distance_point_to_line(point, line):
    
    """
    Calculate the minimum distance from a point to a line in 2D space (slice plane).

    Input:
    point: (x, y) of the point
    line: list with length 2, containing (x, y) coordinates of two points making up the line

    Output:
    distance in mm
    """

    import numpy as np

    p = np.array(point)
    a = np.array(line[0])
    b = np.array(line[1])
    ab = b - a
    ap = p - a
    ab_ap_dot = np.dot(ap, ab)
    ab_norm_sq = np.dot(ab, ab)
    
    if ab_norm_sq == 0:  # Degenerate segment, both endpoints are the same
        return np.linalg.norm(p - a)
    
    # Projection parameter of point P onto the line AB
    t = ab_ap_dot / ab_norm_sq
    
    # Determine the closest point on the line segment
    if t < 0.0:
        closest = a   # Point A is the nearest
    elif t > 1.0:
        closest = b   # Point B is the nearest
    else:
        closest = a + t * ab  # Projection point is the nearest
    
    # Return the distance between the closest point and point P
    return np.linalg.norm(p - closest)
########## END OF distance_point_to_line ##########


########## STRT OF find_closest_line ##########
def find_closest_line(point, contours):

    """
    Find the closest outer edge line in the slice plane to a point on the matrix_map

    Input:
    point: (x, y) of a point being populated on the matrix_map
    contours: a list, each index is a 2x2 ndarray representing the (x, y) of two connected points

    Output:
    ind: index of the closest outer ledge line in contours, essential to locate corresponding normal vector
    closest_line: 2x2 array representing the (x, y) of the two points making up the closest line
    """

    min_distance = float('inf')
    closest_line = None
    for min_ind, line in enumerate(contours):
        dist = distance_point_to_line(point, line)
        if dist < min_distance:
            min_distance = dist
            closest_line = line
            ind = min_ind
    return ind, closest_line
########## END OF find_closest_line ##########


########## START OF solid_or_not ##########
def solid_or_not(point, line, normal):

    """
    Determine if the point being populated on matrix_map is solid

    Input:
    point: (x, y) of a point being populated on the matrix_map
    line: 2x2 array representing the (x, y) of the two points making up the closest line to the point
    normal: corresponding normal vector of the line <x, y, z> only <x, y> are used for calculation

    Output:
    True: solid
    False: not solid
    """

    import numpy as np

    vector = point - line[0]
    result = np.dot(vector, normal[0:2])

    if result < 0 or result == 0:
        return True
    else:
        return False
########## END OF solid_or_not ##########

########## START OF matrix_map ##########
def build_map(contours, normal_vector_contour, line_width):

    """
    Generate a matrix map of the sliced layer.

    Input:
    contours: a list, each index is a 2x2 ndarray representing the (x, y) of two connected points
    normal_vector_contour: a list, same length as connected_points, each index is the normal vector of the corresponding connected points
    liner_width: user input, required line width in mm

    Output:
    map: a matrix map representing the gemoetry of the sliced layer, black is solid, white is empty space
    """

    import numpy as np

    contours = np.array(contours)
    
    reshaped_connected_points = np.reshape(contours, [np.shape(contours)[0]*np.shape(contours)[1], 2])

    x_min = np.min(reshaped_connected_points[:, 0])
    x_max = np.max(reshaped_connected_points[:, 0])
    y_min = np.min(reshaped_connected_points[:, 1])
    y_max = np.max(reshaped_connected_points[:, 1])

    x_pixel = (x_max - x_min)/line_width
    y_pixel = (y_max - y_min)/line_width

    # x_pixel = 100
    # y_pixel = 100

    x = np.linspace(x_min, y_max, int(x_pixel))
    y = np.linspace(y_min, y_max, int(y_pixel))

    X, Y = np.meshgrid(x, y)
    matrix_map = np.zeros(X.shape)

    # Populate binary map
    for ix in range(X.shape[0]):
        for iy in range (Y.shape[0]):
            point = np.array([X[ix, iy], Y[ix, iy]]) # point on the binary map
            ind, closest_line = find_closest_line(point, contours)
            if solid_or_not(point, closest_line, normal_vector_contour[ind]):
                    matrix_map[ix, iy] = 1  # Mark inside region
            # else: # for debugging
                # ind, closest_line = find_closest_line(point, contours, normal_vector_contour)
            # plt.imshow(binary_map, extent=(x_min, x_max, y_min, y_max), origin='lower', cmap='binary')

    return matrix_map
########## END OF matrix_map ##########



