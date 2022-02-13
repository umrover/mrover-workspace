import math
import numpy as np

'''
Generally useful conversion functions
'''

EARTH_RADIUS_M = 6371000


def lat2meters(lat, ref_lat=0):
    '''
    Converts degrees latitude to meters North/South using a nearby ref_lat to
    reduce conversion error

    @param float/ndarray lat: absolute degrees latitude
    @optional ref_lat: reference latitude used to reduce conversion error
    @return float/ndarray: meters North/South relative to ref_lat
    '''
    return np.radians(lat - ref_lat) * EARTH_RADIUS_M


def meters2lat(meters, ref_lat=0):
    '''
    Converts meters North/South to degrees latitude, adding ref_lat to counteract
    its use in lat2meters

    @param float/ndarray meters: meters North/South relative to ref_lat
    @optional float ref_lat: reference latitude used to reduce conversion error
    @return float/ndarray: absolute degrees latitude
    '''
    return np.degrees(meters / EARTH_RADIUS_M) + ref_lat


def long2meters(long, lat, ref_long=0):
    '''
    Converts degrees longitude to meters East/West using a nearby ref_long to reduce
    conversion error

    @param float/ndarray long: absolute degrees longitude
    @param float/ndarray lat: absolute degrees latitude
    @optional ref_long: reference longitude used to reduce conversion error
    @return float/ndarray: meters East/West relative to ref_long
    '''
    lat_cos = np.cos(np.radians(lat))
    scaled_long = np.multiply(long - ref_long, lat_cos)
    return np.radians(scaled_long) * EARTH_RADIUS_M


def meters2long(meters, lat, ref_long=0):
    '''
    Converts meters East/West to degrees longitude, adding ref_long to counteract its
    use in long2meters

    @param float/ndarray meters: meters East/West relative to ref_long
    @param float/ndarray lat: absolute degrees latitude
    @optional float ref_long: reference longitude used to reduce conversion error
    @return float/ndarray: absolute degrees longitude
    '''
    lat_cos = np.cos(np.radians(lat))
    scaled_long = np.degrees(meters / EARTH_RADIUS_M)
    return (scaled_long / lat_cos) + ref_long


def decimal2min(decimal):
    '''
    Converts decimal degrees to integer degrees and decimal minutes

    @param float decimal: decimal degrees
    @return int: integer degrees
    @return float: decimal minutes
    '''
    min, deg = math.modf(decimal)
    return int(deg), min * 60


def min2decimal(deg, min):
    '''
    Converts integer degrees and decimal minutes to decimal degrees

    @param int deg:  integer degrees
    @param float min: decimal minutes
    @return float: decimal degrees
    '''
    return deg + min / 60
