from BotSlicer import BotSlice

from generate_gcode import save_gcode

layer_thickness = 0.5 # mm
line_width = 1 # mm
gantry_speed = 40 # mm/min

stl_path = ''


output = BotSlice(stl_path, layer_thickness, line_width, gantry_speed)

save_gcode(output, 'test_output')
