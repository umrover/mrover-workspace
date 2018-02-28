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

export function inv_projection({center_lat, center_lon}, {x, y}) {
    const center_lat_r = center_lat * (Math.PI / 180);
    const center_lon_r = center_lon * (Math.PI / 180);
    let theta_prime = Math.atan2(y, x);
    if (theta_prime < 0) {
        theta_prime += 2.0*Math.PI;
    }
    let bearing = Math.PI/2.0 - theta_prime;
    if (bearing < 0) {
        bearing += 2.0*Math.PI;
    }
    const dist = Math.sqrt(x*x + y*y);
    const dlat_meters = dist * Math.cos(bearing);
    const dlat = Math.asin(dlat_meters/EARTH_RAD);
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

export function projection({center_lat, center_lon}, {lat, lon}) {
    const center_lat_r = center_lat * (Math.PI / 180);
    const center_lon_r = center_lon * (Math.PI / 180);
    const coord_lat = lat * (Math.PI / 180);
    const coord_lon = lon * (Math.PI / 180);
    const dlat = coord_lat - center_lat_r;
    const dlon = (coord_lon - center_lon_r) * Math.cos((center_lat_r + coord_lat)/2);
    const dist = Math.sqrt(dlon*dlon + dlat*dlat) * EARTH_RAD;

    const dlat_meters = EARTH_RAD * Math.sin(dlat);
    let bearing = Math.acos(dlat_meters / dist);
    if (center_lon_r > coord_lon) {
        bearing = 2.0 * Math.PI - bearing;
    }
    if (dlat_meters < 1 && dlat_meters > -1) {
        if (center_lon_r < coord_lon) {
            bearing = Math.PI/2.0;
        } else {
            bearing = 3.0*Math.PI/2.0;
        }
    }

    const theta_prime = Math.PI/2.0 - bearing;
    return {
        x: dist * Math.cos(theta_prime),
        y: dist * Math.sin(theta_prime)
    };
};
