# requires PyGCode: pip3 install pygcode

from pygcode import GCodeRapidMove, GCodeLinearMove
from pygcode import Machine, Line, split_gcodes
m = Machine()
coordinates = []
pairs = []
with open('triangle.gcode', 'r') as fh:
        for line_text in fh.readlines():
                line = Line(line_text)
                line.block.gcodes  # list of gcodes
                if line.block.gcodes: # not a blank line
                        (befores, (g,), afters) = split_gcodes(line.block.gcodes, (GCodeRapidMove, GCodeLinearMove))
                        if g.X is not None:
                            pairs = [g.X-150,g.Y]
                            coordinates.append(pairs)
print("Number of points:", len(coordinates))
print(coordinates)
with open('triangle.json', 'w') as fh:
        fh.write(repr(coordinates))

i = 0
for line in coordinates:        
        print("G1","X",coordinates[i][0],"Y",coordinates[i][1],"F2.0")
        i = i + 1
