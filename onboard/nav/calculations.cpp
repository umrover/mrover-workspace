#include "calculations.hpp"

inline double degree_to_radian(const double x, const double minute) {
    return (PI / 180) * (x + minute / 60.0);
}

inline double radian_to_degree(const double x) {
    return x * 180 / PI;
}
							
double estimate_noneuclid(const rover_msgs::Odometry &start, 
								  const rover_msgs::Odometry &dest){
    double start_lat = degree_to_radian(start.latitude_deg, start.latitude_min);
    double start_long = degree_to_radian(start.longitude_deg, start.longitude_min); 
    double dest_lat = degree_to_radian(dest.latitude_deg, dest.latitude_min);
    double dest_long = degree_to_radian(dest.longitude_deg, dest.longitude_min);

    double dlon = (dest_long - start_long) * cos((start_lat + dest_lat) / 2);
    dlon *= dlon;
    double dlat = (dest_lat - start_lat);
    dlat *= dlat;
    return sqrt(dlon + dlat)*earthRadiusMeters;
}

double calc_bearing(const rover_msgs::Odometry & start, 
                            const rover_msgs::Odometry & dest) {
    double start_lat = degree_to_radian(start.latitude_deg, start.latitude_min);
    double start_long = degree_to_radian(start.longitude_deg, start.longitude_min); 
    double dest_lat = degree_to_radian(dest.latitude_deg, dest.latitude_min);
    double dest_long = degree_to_radian(dest.longitude_deg, dest.longitude_min);


    double vert_component_dist = earthRadiusMeters * std::sin(dest_lat - start_lat);
    double noneuc_dist = estimate_noneuclid(start,dest);

    double bearing_phi = std::acos(vert_component_dist / noneuc_dist);
    if (start_long > dest_long) bearing_phi = 2 * PI - bearing_phi;

    if(vert_component_dist < 0.001 && vert_component_dist > -0.001){
        if (start_long < dest_long) {
            bearing_phi = PI / 2.0;
        } else {
            bearing_phi = 3.0 * PI / 2.0;
        }
    }

    return radian_to_degree(bearing_phi);
}

bool within_threshold(const rover_msgs::Odometry &odom1, 
                              const rover_msgs::Odometry &odom2, 
                              const double threshold) {
    return estimate_noneuclid(odom1,odom2) <= threshold;
}
