def rotate(model, axis, theta_degrees):
    
    import numpy as np
    
    # Function: rotate a model around defined axis by defined degrees
    
    # Input: 
    # model: from mesh.Mesh.from_file(filename)
    # axis: 'x', 'y', 'z'
    # theta_degress: rotation in degree
    
    # Output:
    # model: rotated model
          
    # Convert to radian
    theta = np.radians(theta_degrees)
    
    # Construct rotation matrix    
    if axis == 'x':
        rot_matrix = np.array([[1, 0, 0],
                                   [0, np.cos(theta), -np.sin(theta)],
                                   [0, np.sin(theta), np.cos(theta)]])
    elif axis == 'y':
        rot_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                   [0, 1, 0],
                                   [-np.sin(theta), 0, np.cos(theta)]])
    elif axis == 'z':
        rot_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                   [np.sin(theta), np.cos(theta), 0],
                                   [0, 0, 1]])
    else:
        raise ValueError("Invalid rotation axis.")

    # Rotate
    model.vectors = np.dot(model.vectors.reshape(-1, 3), rot_matrix.T).reshape(-1, 3, 3)
    
    return model
    
def optimal_orientation(model):
    
    # still under work
    
    import numpy as np
        
    volume, cog, inertia = model.get_mass_properties()
                
    # Determine which side of the bounding box is closest to the COG
    max_dim = np.argmax(np.abs(cog - model.min_))
        
    # Assuming we rotate around 'x' or 'y' to align the 'z' dimension with the plane
    if max_dim == 0:  # x has the highest difference
        model.rotate('y', 90 if cog[max_dim] > 0 else -90)
    elif max_dim == 1:  # y has the highest difference
        model.rotate('x', -90 if cog[max_dim] > 0 else 90)
    # No need to rotate if Z is the highest since we assume Z is already aligning with the XY plane
        
    return model

def translate_to_origin(model):
    
    # Function: translate the model so it starts closest to the origin
    # and is placed in positive xyz direction
    
    # Input: model
    
    # Output: translated model
    
    import numpy as np
    
    # Place the vertice of the model at (0, 0, 0)
    
    # Find the minimum values along each axis
    min_x, min_y, min_z = np.min(model.x), np.min(model.y), np.min(model.z)
    
    # Calculate the translation needed to move the mesh to the origin
    translation = np.array([min_x, min_y, min_z])
    
    # Apply the translation to each vertex of the mesh
    for i in range(len(model.points)):
        for j in range(3):  # Each triangle has 3 vertices
            model.points[i][j*3:j*3+3] -= translation
    
    return model
