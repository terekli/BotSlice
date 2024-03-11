def BotSlice(filename, layer_thickness, line_width, gantry_speed):
    
    import numpy as np
    from stl import mesh
    
    from visualize import visualize_before_slicing
    from visualize import visualize_contour
    
    from mesh_manipulator import rotate
    from mesh_manipulator import translate_to_origin
    
    from get_edge_points import get_edge_points
    from create_contour import build_contours
    
    from generate_gcode import generate_start_gcode
    from generate_gcode import generate_end_gcode
    
    # Initiate GCode
    gcode = []
    gcode = generate_start_gcode(gcode)

    # Load the STL file
    model = mesh.Mesh.from_file(filename) # unit: cm
    
    # Check if model is closed
    # if ~model.is_closed():
        # raise TypeError('STL model is not closed')
        
    # Translate the model to (0, 0, 0)
    model = translate_to_origin(model)
    
    # Rotate the model as you see fit
    # model = rotate(model, 'x', 90)
    
    model.dimension = model.max_ - model.min_
    
    visualize_before_slicing(model)

    # Calculate how many layers to print
    total_layers = int(np.round(model.dimension[2] / layer_thickness, 0))
    current_layer = 1
    
    # Slice
    while current_layer <= total_layers:
        
        # Height of the printed model before printing current layer
        current_layer_height = (current_layer-1) * layer_thickness
        
        # Calculate edge points at the current slice plane
        connected_points, normal_vector = get_edge_points(model, current_layer_height)

        # Organize the connected points into contour
        contour = build_contours(connected_points)
        
        # Visualize the sliced contour
        visualize_contour(contour, current_layer)
        
        current_layer += 1   
        
    gcode = generate_end_gcode(gcode)

        
    return gcode
