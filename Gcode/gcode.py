# requires PyGCode: pip3 install pygcode

from pygcode import GCodeRapidMove, GCodeLinearMove
from pygcode import Machine, Line, split_gcodes
m = Machine()
coordinates = []
pairs = []
with open('triangle.gcode', 'r') as fh:
        for line_text in fh.readlines():
                line = Line(line_text)
                line.block.gcodes  # is your list of gcodes
                if line.block.gcodes: # not a blank line
                        line.block.modal_params  # are all parameters not assigned to a gcode, assumed to be motion modal parameters
                        (befores, (g,), afters) = split_gcodes(line.block.gcodes, (GCodeRapidMove, GCodeLinearMove))
                        if g.X is not None:
                            pairs = [[g.X],[g.Y]]
                            coordinates.append(pairs)
print("Number of points:", len(coordinates))
print(coordinates)
