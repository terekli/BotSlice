# BotSlice

## Intro
This is a slicer software written and optimized for 3D printing via Direct-Ink-Writing (DIW).  
For more information about DIW please refer to the following published paper:  
* Li, T.& Naguib, H. E. et al (2023). Additive Manufacturing, 74.  
* Li, T. & Naguib, H. E. et al (2023). Nano Energy, 108909  

This software is continously under development, if you observed any bug or have suggetions for improvement, feel free to reach out to me at: https://www.linkedin.com/in/terekli/

## Current State
The code takes in standard STL file as input, and generate contours at each slicing layer defined by another layer_thickness input.
A figure is generated for each sliced layer, showcasing all the individual closed contours.
Normal vector is then used to create a matrix map, identifying the solid geometry.

## Demo
Rendered illustration of a grid structure STL input with 1 mm thickness:

![Rendered illustration of a grid structure STL input with 1 mm thickness:](/stl_model.png)

Sliced contour of the fifth layer, each closed contours are shown in unique color:

![Slice layer showing individual contours:](/individual_contour.png)

Sliced layer map, 1 represents solid:
![Slice layer showing solid geometry:](/layer_map.png)


## Run the Code
In [run.py](run.py):
* stl_path: directory of your STL model
* layer_thickness: required layer thickness in mm
* line_width: required line width in mm
* gantry_speed: required printing speed in mm/min

Ouput GCode file is saved in the same directory.

You can change the default orientaiton of your model before slicing.
* In [BotSlicer.py](BotSlicer.py):
```
# Rotate the model as you see fit
model = rotate(model, 'x', 90)
```



