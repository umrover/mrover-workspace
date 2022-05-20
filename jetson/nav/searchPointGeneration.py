import math
import numpy as np
import matplotlib.pyplot as plt
import enum

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

def generateSquareSpiralInward (points, distance):
    return generateSquareSpiral(points, distance)[::-1] 

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

class SearchType(enum.Enum):
    point_equidistant_spiral = '0'
    radially_equidistant_spiral = '1'
    square_spiral = '2'

print("\n-----Search types-----\nRadially Equidistant Spiral: 0\nPoint Equidistant Spiral: 1\nSquare Spiral: 2\n")
search_type = input("Select a search type: ")
if search_type == SearchType.point_equidistant_spiral.value: # POINT EQUIDISTANCE SPIRAL
    # generateSpiralSearchPoints 
    #           (Radius of spiral, Number of Points, Number of coils)
    radius = float(input("Enter a radius: "))
    sides = int(input("Enter the number of points: "))
    coils = int(input("Enter the number of coils: "))
    coords = generateSpiralSearchPoints(radius, sides, coils) # Try (20, 200, 10) to see basic example
elif search_type == SearchType.radially_equidistant_spiral.value: # RADIALLY EQUIDISTANT SPIRAL
    # generateSpiralSearchPoints 
    #           (Radius of spiral, Distance between points, Number of coils, Rotation from start)
    radius = float(input("Enter a radius: "))
    distance = float(input("Enter distance between points: "))
    coils = int(input("Enter number of coils: "))
    rotation = float(input("Enter rotation of spiral start (degrees): "))
    coords = generateEquidistantSpiralSearchPoints(radius, distance, coils, rotation) # Try (20, 1.2, 10, 90) to see basic example
elif search_type == SearchType.square_spiral.value: # SQUARE SPIRAL SEARCH
    # generateSquareSpiral
    #           (number of points, distance between each coil of the spiral)
    number_of_points = int(input("Enter Number of Points: "))
    distance = float(input("Enter Distance Between Points: "))
    coords = generateSquareSpiral(number_of_points, distance) # Currently, we use (13,3) as arguements
    spiral_in = (input("Do You Want to Spiral In? (y/n)") == 'y')
    if spiral_in:
        coords += generateSquareSpiralInward(number_of_points, distance)
else:
    print ("Not a valid type")
    exit(1)
show_coords = input("Do you want to display the search coordinates? (y/n)")
if show_coords == 'y':
    showCoords(coords)

custom_name = input("Enter filename (n for default):")
name = 'spiral_search_points.txt'
if custom_name != 'n':
    name = custom_name

with open('jetson/nav/search/'+name, 'w') as f:
    # print (len(coords), file=f)
    for x,y in coords:
        polar_coord = cart2pol(x,y)
        # (Rho (distance), Phi (angle))
        print(polar_coord[0], polar_coord[1], file=f)