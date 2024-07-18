def BotSlice(filename, layer_thickness, line_width, gantry_speed):
    
    import numpy as np
    from stl import mesh
    
    from visualize import visualize_before_slicing
    from visualize import visualize_contours
    from visualize import visualize_pairs
    from visualize import visualize_map

    from manipulate import rotate
    from manipulate import translate_to_origin
    
    from layer import get_edge_points
    from layer import build_contours
    from layer import build_map
    
    from gcode import generate_start_gcode
    from gcode import generate_end_gcode
    
    # from temp import build_contours
    
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
    
    # visualize_before_slicing(model)

    # Calculate how many layers to print
    total_layers = int(np.round(model.dimension[2] / layer_thickness, 0))
    current_layer = 1
    
    # Slice
    while current_layer <= total_layers:
        
        # Height of the printed model before printing current layer
        current_layer_height = (current_layer-1) * layer_thickness
        
        # Calculate edge points at the current slice plane
        connected_points, normal_vector = get_edge_points(model, current_layer_height)

        # Visualize_pairs(connected_points, current_layer)

        # Organize the connected points into contour
        contours, normal_vector_contour = build_contours(connected_points, normal_vector)
        
        # Visualize the sliced contour
        # visualize_contours(contours, current_layer)

        # Create Binary Map
        map = build_map(contours, normal_vector_contour, line_width)

        # Visualize
        visualize_map(map)
        
        current_layer += 1   
        
    gcode = generate_end_gcode(gcode)

        
    return gcode
