########## START OF close_to ##########

def relative_distance(pair, destination):
    
    import numpy as np
    
    # Function:
    # Find which points in a pair of points is closest to the destination
    
    # Input:
    # pair: pair of points of ndarray size (2,2)  
    # destination: destination coordinate (x,y)
    
    # Output:
    # 1x2 array pointer on the order of closest to destination
    # 0 is closer to the destination
    # 1 is further to the destination
    
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
    
    # Function:
    # Find closest points in connected_points to the last point in contour
    
    # Input:
    # contours
    # connected_points
    
    # Output:
    # ind: index of closest pairs in connected_points
    
    last_point = contours[-1]
    
    # Find the ind of connected_points that contain the last point in contour
    for index, pair in enumerate(connected_points):
        # Check if the point is in the entry
        if (pair == [last_point[0], last_point[1]]).all(axis=1).any():
            return index

 ########## END OF closest_pair ##########



########## START OF build_contours ##########

def build_contours(connected_points):
    import numpy as np
        
    # Remove entries that are triangle or single point
    connected_points = [entry for entry in connected_points if entry.shape != (1, 2) and entry.shape != (3, 2)]
      
    # Inititae
    contours = []
    contours.append(np.array([np.nan, np.nan]))

    # Whether the contour is closed
    closed = True

    # Process the pairs
    while len(connected_points) != 0:
        
        if closed is False:
            # Find closest point in connected_points to last point in contours
            ind = closest_pair(contours, connected_points)
            
            # If no closest pair is found, which often occurs for complex
            # or corrupted model
            if ind is None:
                a = 1
                
            temp = relative_distance(connected_points[ind], contours[-1])
            
            if temp is None:
                a = 1
                
            contours.append(connected_points[ind][temp[1],:])
            connected_points.pop(ind)
        else:
            # Start a new contour with the first pair in connected_points
            closed = False
            contours_start = connected_points[0][0]
            contours.append(contours_start)
            contours.append(connected_points[0][1])
            connected_points.pop(0)
        
        # Check if a closed contour is formed
        if np.array_equal(contours_start, contours[-1]):
            closed = True
            # Each closed contour is seperated by [nan, nan]
            contours.append(np.array([np.nan, np.nan]))
            
    # Remove consecutive entries that are the same
    # This generate accurate number of closed contours
    contours = [contours[i] for i in range(len(contours)) if i == 0 or not np.array_equal(contours[i], contours[i-1])]
        
    return np.array(contours)  # Convert list of points to an n*2 NumPy array
 
    ########## END OF build_contour ##########


