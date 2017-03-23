from subprocess import check_output, Popen, PIPE, STDOUT
import StringIO
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

def GenPoints(lines):
    """
    Create a line using points list
    length is the number of points for the line.
    lines is the points list.
    """
    ll = len(lines)
    linePoints = np.empty((3, ll))
    for index in range(0, ll):
        line = lines[index].rstrip().split(',')
        linePoints[0][index] = float(line[0])
        linePoints[1][index] = float(line[1])
        linePoints[2][index] = float(line[2])
    return linePoints

def UpdateLines(num, dataLines, lines):
    # for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
    lines.set_data(dataLines[0:2, :num])
    lines.set_3d_properties(dataLines[2, :num])
    # return lines

# read stdout from target program
# ps = Popen(('./silhouette line/line-1d-seg-1490089995240.csv'), shell=True, stdout=PIPE, stderr=STDOUT)
ps = Popen(('./silhouette test/01.csv'), shell=True, stdout=PIPE, stderr=STDOUT)
# ps = Popen(('./silhouette line/line-quad-paral-1490238688927.csv'), shell=True, stdout=PIPE, stderr=STDOUT)
output, err = ps.communicate()

# readline from the output string
buf = StringIO.StringIO(output)
lines = buf.readlines()
points = GenPoints(lines)

fig = plt.figure()
ax = p3.Axes3D(fig)
axlines = ax.plot(points[0, 0:1], points[1, 0:1], points[2, 0:1])[0]

# Setting the axes properties
axUpper = int(np.max(points) + 1)
axLower = int(np.min(points) - 1)
ax.set_xlim3d([axLower, axUpper])
ax.set_xlabel('X')

ax.set_ylim3d([axLower, axUpper])
ax.set_ylabel('Y')

ax.set_zlim3d([axLower, axUpper])
ax.set_zlabel('Z')

ax.set_title('3D Test')

if len(points) != 0:
    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, UpdateLines, len(lines), fargs=(points, axlines),
                                   interval=50, blit=False)

plt.show()
