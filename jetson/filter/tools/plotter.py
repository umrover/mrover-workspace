import numpy as np
import os
import math
import sys
import matplotlib.pyplot as plot
import statistics
import datetime
from src.conversions import lat2meters, long2meters, meters2lat, meters2long

REF_LAT = 42.2766
REF_LONG = -83.7382

class Plotter:

    def readCsv(self, type, dtype, names_in):
        # Reads in the CSV file specified by log

        path_self = os.path.abspath(os.path.dirname(__file__))
        file_path = os.path.join(path_self, '../logs/')

        # Read data
        if names_in:
            self.data = np.genfromtxt(file_path + type + 'Log.csv',
                                     delimiter=',', names=True, dtype=dtype)
        else:
            self.data = np.genfromtxt(file_path + type + 'Log.csv',
                                     delimiter=',', dtype=dtype)

    def plotStatic(self, subplot_loc):
        # Plots the coordinates from data in the specified subplot

        # Convert DMS to decimal
        long = self.data['long_deg'] + self.data['long_min']/60
        lat = self.data['lat_deg'] + self.data['lat_min']/60

        mean_lat = np.mean(lat)
        mean_long = np.mean(long)

        # Convert to meters
        long = long2meters(long, lat, REF_LONG)
        lat = lat2meters(lat, REF_LAT)

        mean_long = long2meters(mean_long, mean_lat, REF_LONG)
        mean_lat = lat2meters(mean_lat, REF_LAT)

        # Calculate delta vectors
        delta_long = [i - mean_long for i in long]
        delta_lat = [i - mean_lat for i in lat]

        # Calculate error circles
        sigma_long = np.std(delta_long)
        sigma_lat = np.std(delta_lat)
        cep = 0.56*sigma_long + 0.62*sigma_lat
        _2drms = 2*math.sqrt(sigma_long*sigma_long + sigma_lat*sigma_lat)

        # Plot
        subplot = plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        plot.scatter(delta_long, delta_lat)

        plot.Circle((0, 0), radius=cep, color='r', fill=False, label='CEP')
        plot.Circle((0, 0), radius=_2drms, color='g', fill=False, label='2DRMS')

        _min = min(np.amin(delta_long)-sigma_long,
                   np.amin(delta_lat)-sigma_lat, -_2drms)
        _max = max(np.amax(delta_long)+sigma_long,
                   np.amax(delta_lat)+sigma_long, _2drms)

        plot.axis([_min, _max, _min, _max])
        plot.xticks(rotation=60)
        plot.xlabel('Delta Longitude (meters)')
        plot.ylabel('Delta Latitude (meters)')
        plot.title('GPS Coordinates')

        subplot.text(0.5, -0.15, 'Long-Variance: ' + str(np.var(long)) +
                  '\nLat-Variance ' + str(np.var(lat)) + '\nLong-Mean: ' +
                  str(meters2long(np.mean(long), 42.277)) + '\nLat-Mean: ' +
                  str(meters2lat(np.mean(lat))), ha='center', transform=subplot.transAxes)

        plot.legend()

    def plotSpeed(self, subplot_loc):
        # Plots the speed from data in the specified subplot

        # Plot
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        plot.plot(range(0, self.data.shape[0], 1), self.data['speed'])
        plot.axis([0, self.data.shape[0], np.amin(self.data['speed']),
                  np.amax(self.data['speed'])])
        plot.xlabel('Time (seconds)')
        plot.ylabel('Speed (m/s)')
        plot.title('Speed')

    def plotBearing(self, subplot_loc):
        # Plots the bearing from data in the specified subplot

        # Plot
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        shifted_bearing = np.array([i - 360 if i > 180 else i for i in self.data['bearing']])
        plot.plot(range(0, self.data.shape[0], 1), shifted_bearing)
        plot.axis([0, self.data.shape[0], np.amin(shifted_bearing),
                  np.amax(shifted_bearing)])
        plot.xlabel('Time (seconds)')
        plot.ylabel('Bearing (degrees)')
        plot.title('Bearing')
    
    def plotPath(self, color, label, subplot_loc):
        # Convert DMS to decimal
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        long = self.data['long_deg'] + self.data['long_min']/60
        lat = self.data['lat_deg'] + self.data['lat_min']/60

        # Convert to meters
        long = long2meters(long, lat, REF_LONG)
        lat = lat2meters(lat, REF_LAT)

        return plot.scatter(long, lat, color=color, label=label)

    def plot(self, data_type):
        # Decides what to plot

        title = data_type + ' ' + str(datetime.date.today())

        if data_type == 'gps':
            self.readCsv('gps', float, True)
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        elif data_type == 'phone':
            self.readCsv('phone', float, True)
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        # elif data_type == 'imu':
            # self.plotImu()
        elif data_type == 'truth':
            self.readCsv('truth', float, True)
            self.plotPath('black','truth',[1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        elif data_type == 'odom':
            self.readCsv('odom', float, True)
            self.plotPath('black','odom',[1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        elif data_type == 'newGps':
            self.readCsv('newGps', float, True)
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        elif data_type == 'filterSim':
            self.readCsv('gps', float, True)
            self.plotPath('red', 'gps', [1, 1, 1])
            self.readCsv('truth', float, True)
            self.plotPath('black', 'truth', [1, 1, 1])
            self.readCsv('movAvg', float, True)
            self.plotPath('green', 'moving average', [1, 1, 1])
            self.readCsv('odom', float, True)
            self.plotPath('blue', 'kalman filter', [1, 1, 1])

            plot.xlabel('Latitude (meters)')
            plot.ylabel('Longitude (meters)')

            self.readCsv('config', str, False)
            title += '\nParams: '
            for i in range(self.data.shape[1]):
                title += self.data[0][i] + '=' + self.data[1][i] + ', '

            plot.legend(loc='upper right')
        else:
            print('Invalid data type.')
            sys.exit()

        plot.tight_layout()
        plot.suptitle(title)
        plot.show()


if __name__ == "__main__":
    # Get arguments
    if len(sys.argv) != 2:
        print('Error: Usage from onboard/filter is python3 -m tools.plotter <data_type>')
        sys.exit()

    # Plot
    plotter = Plotter()
    plotter.plot(sys.argv[1])