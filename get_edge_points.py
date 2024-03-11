def get_edge_points(model, slice_z):
    
    # Function: calculate the edge points of the model at a certain slice plane
    
    # Input: 
    # model from mesh.Mesh.from_file(filename)
    # slice_z: height of the slice plane
    
    # Output: 
    # edge_points, each row represent the (x,y) or edge points at the slice plane
    # solid_directions: each row represent the solid direction of each edge points
    
    import numpy as np
    from find_intersection import find_intersection

    connected_points = []
    normal = []
    
    # Loop through each triangle
    for itriangle in range(np.shape(model.vectors)[0]):
        
        # grab the triangle
        triangle = model.vectors[itriangle,:,:]
        
        # Only try to find edge points if triangle intersect the slicing plane
        # and is above the slicing plane
        if np.min(triangle[:,2]) <= slice_z <= np.max(triangle[:,2]):

            connected_points.append(find_intersection(triangle, slice_z))
            normal.append(model.normals[itriangle,:])
            
    return connected_points, normal
           
