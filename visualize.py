########## START OF visualize_before_slicing ##########

def visualize_before_slicing(model):
    
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import matplotlib.pyplot as plt
    import numpy as np
    
    # Function: visualize the 3D model
    
    # Input: model from mesh.Mesh.from_file(filename)
    
    # Extract the vectors (triangles)
    polygons = [face for face in model.vectors]
    # Now, use the polygons with Poly3DCollection
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = Poly3DCollection(polygons, linewidths=0.5, edgecolors='k', alpha=0.5)
    ax.add_collection3d(collection)
    # Calculate the range for each dimension to set the plot limits
    x_range = [np.min(model.vectors[:,:,0]), np.max(model.vectors[:,:,0])]
    y_range = [np.min(model.vectors[:,:,1]), np.max(model.vectors[:,:,1])]
    z_range = [np.min(model.vectors[:,:,2]), np.max(model.vectors[:,:,2])]
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)
    # Labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Show the plot
    plt.show()
    
########## END OF visualize_before_slicing ##########



########## START OF visualize_contours ##########
def visualize_contours(contours, current_layer):
    
    import numpy as np
    import matplotlib.pyplot as plt
    
    # Function: visualize the sliced contour of each layer
    # Dots represent edge points
    # Lines represent connected edge points
    
    # Input: 
    # Contours
    
    for points in contours:
        plt.plot(points[:,0], points[:,1], 'o-')
    plt.title(f'Layer {current_layer}')
    
    plt.show()
########## END OF visualize_contours ##########
    


########## START OF visualize_pairs ##########
def visualize_pairs(connected_points, current_layer): 
    
    # Function: visualize the dual (x,y) edge point of the same triangle
    #           connected without generating contour
    
    # Input: 
    # connected_points
    # current_layer

    import numpy as np
    import matplotlib.pyplot as plt

    for combination in connected_points:
        if np.size(combination,0)==2:
            x = combination[:, 0]
            y = combination[:, 1]
            plt.plot(x, y, marker='o', color='k')
            plt.title(f'Layer {current_layer}')

    plt.show()
########## END OF visualize_pairs ##########



########## START OF visualize_map ##########
def visualize_map(matrix_map): 
    import matplotlib.pyplot as plt


    plt.imshow(matrix_map, origin='lower')
    plt.colorbar()
    plt.title('Map of Slice Plane')
    plt.show()
########## END OF visualize_map ##########
