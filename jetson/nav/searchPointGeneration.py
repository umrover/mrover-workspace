import math
import numpy as np
import matplotlib.pyplot as plt

def generateSpiralSearchPoints(radius, sides, coils):
    awayStep = radius/sides
    aroundStep = coils/sides
    aroundRadians = aroundStep * 2 * math.pi
    coordinates = [(0,0)]
    for i in range(1, sides+1):
        away = i * awayStep
        around = i * aroundRadians
        x =  math.cos(around) * away
        y = math.sin(around) * away
        coordinates.append((x,y))
    return coordinates
    
def generateEquidistantSpiralSearchPoints(radius, distance, coils, rotation):
    thetaMax = coils * 2 * math.pi
    awayStep = radius / thetaMax
    theta = distance / awayStep
    coordinates = [(0,0)]
    while theta <= thetaMax:
        away = awayStep * theta
        around = theta + rotation
        x = math.cos(around) * away
        y = math.sin(around) * away
        coordinates.append((x,y))
        theta += distance / away
    return coordinates

def generateSquareSpiral (points, distance):
    directions = [(0,1), (1,0), (0,-1), (-1,0)]
    coordinates = [(0,0)]
    new_distance = distance
    for i in range(0,points):
        coordinates.append( (coordinates[-1][0]+new_distance*directions[i%4][0], coordinates[-1][1]+new_distance*directions[i%4][1]) )
        new_distance += (i%2)*distance
    return coordinates

def showCoords(coordinates):
    x_coords = [i[0] for i in coordinates]
    y_coords = [i[1] for i in coordinates]
    plt.scatter(x_coords, y_coords)
    for i in range(1, len(x_coords)):
        plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'ro-')
    plt.show()

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = -1 * (np.arctan2(y, x) * 180 / math.pi - 90)
    return(rho, phi)

print("\n-----Search types-----\nRadially Equidistant Spiral: 0\nPoint Equidistant Spiral: 1\nSquare Spiral: 2\n")
search_type = input("Select a search type: ")
if search_type == '0': # POINT EQUIDISTANCE SPIRAL
    # generateSpiralSearchPoints 
    #           (Radius of spiral, Number of Points, Number of coils)
    coords = generateSpiralSearchPoints(20, 200, 10)
elif search_type == '1': # RADIALLY EQUIDISTANT SPIRAL
    # generateSpiralSearchPoints 
    #           (Radius of spiral, Distance between points, Number of coils, Rotation from start)
    coords = generateEquidistantSpiralSearchPoints(20, 1.2, 10, 90)
elif search_type == '2': # SQUARE SPIRAL SEARCH
    # generateSquareSpiral
    #           (number of points, distance between each coil of the spiral)
    coords = generateSquareSpiral(13, 3)
else:
    print ("Not a valid type")
    exit(1)
show_coords = input("Do you want to display the search coordinates? (y/n)")
if show_coords == 'y':
    showCoords(coords)

with open('jetson/nav/search/spiral_search_points.txt', 'w') as f:
    # print (len(coords), file=f)
    for x,y in coords:
        polar_coord = cart2pol(x,y)
        # (Rho (distance), Phi (angle))
        print(polar_coord[0], polar_coord[1], file=f)