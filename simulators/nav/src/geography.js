const EARTH_RAD = 6371000.0;

export function to_dms({lat, lon}) {
    return {
        latitude_deg: Math.trunc(lat),
        latitude_min: (lat % 1)*60,
        longitude_deg: Math.trunc(lon),
        longitude_min: (lon % 1)*60
    };
};

export function to_decimal({latitude_deg, latitude_min, longitude_deg, longitude_min}) {
    return {
        lat: latitude_deg + latitude_min/60.0,
        lon: longitude_deg + longitude_min/60.0
    }
};

// Finds the distance between two odom objects and the bearing between
// them from north. Note: this does not take into account the direction
// that an object (e.g. the rover) is facing.
export function dist_and_bearing(from, to) {
    const from_lat = from.lat*Math.PI/180;
    const from_lon = from.lon*Math.PI/180;
    const to_lat = to.lat*Math.PI/180;
    const to_lon = to.lon*Math.PI/180;

    const dlat = to_lat - from_lat;
    const dlon = (to_lon - from_lon)*Math.cos((to_lat + from_lat)/2.0);
    const dist = Math.sqrt(dlon*dlon + dlat*dlat) * EARTH_RAD;

    const dlat_meters = EARTH_RAD*Math.sin(dlat);
    let bearing = Math.acos(dlat_meters / dist);
    if (from_lon > to_lon) {
        bearing = 2.0*Math.PI - bearing;
    }
    if (dlat_meters < 0.001 && dlat_meters > -0.001) {
        if (from_lon < to_lon) {
            bearing = Math.PI/2.0;
        } else {
            bearing = 3.0*Math.PI/2.0;
        }
    }

    return { dist, bearing };
};

export function inv_projection({center_lat, center_lon}, {x, y}) {
    const center_lat_r = center_lat * (Math.PI / 180);
    const center_lon_r = center_lon * (Math.PI / 180);
    let theta_prime = Math.atan2(y, x);
    let bearing = Math.PI/2.0 - theta_prime;
    const dist = Math.sqrt(x*x + y*y);
    const dlat = Math.asin(y/EARTH_RAD);
    let dlon = Math.sqrt((dist/EARTH_RAD)*(dist/EARTH_RAD) - dlat*dlat);
    if (x < 0) {
        dlon *= -1;
    }

    const coord_lat = dlat + center_lat_r;
    const coord_lon = center_lon_r + dlon/Math.cos((center_lat_r + coord_lat)/2.0);

    return {
        lat: coord_lat * (180.0/Math.PI),
        lon: coord_lon * (180.0/Math.PI)
    };
};

export function projection({center_lat, center_lon}, pt) {
    const { dist, bearing } = dist_and_bearing({
        lat: center_lat,
        lon: center_lon
    }, pt);

    const theta_prime = Math.PI/2.0 - bearing;
    return {
        x: dist * Math.cos(theta_prime),
        y: dist * Math.sin(theta_prime)
    };
};

// Convert an angle from radians on a compass to the
// equivalent on the simulator field which.
// The computer has 0 at "3 o'clock" and goes clockwise
export function compass_to_computer(angle) {
    angle -= Math.PI / 2;
    angle *= -1;
    angle += 2 * Math.PI;
    angle %= 2 * Math.PI;
    return -1 * angle;
};
