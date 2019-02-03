
# class NavState:
#     Off = 0
#     Done = 1
#     Turn = 10
#     Drive = 11
#     SearchFaceNorth = 20
#     SearchFace120 = 21
#     SearchFace240 = 22
#     SearchFace360 = 23
#     SearchTurn = 24
#     SearchDrive = 25
#     TurnToBall = 28
#     DriveToBall = 29
#     TurnAroundObs = 30
#     DriveAroundObs = 31`
#     SearchTurnAroundObs = 32
#     SearchDriveAroundObs = 33
#     Unknown = 255
class FilterClass:
    def __init__(self):

        self.updateFlag = False

        self.stale_raw_imu_ = raw_imu()
        self.stale_raw_gps_ = raw_gps()
        self.stale_raw_enc = raw_wheelenc()
        
        self.fresh_raw_imu_ = raw_imu()
        self.fresh_raw_gps_ = raw_gps()
        self.fresh_raw_enc_ = raw_wheelenc()        
        

        self.clean_bearing = UNDEFINED # Bearing of rover.
        self.clean_gspeed = UNDEFINED
        self.clean_latm , self.clean_latd = UNDEFINED, UNDEFINED
        self.clean_longm, self.clean_longd = UNDEFINED, UNDEFINED


    def filter():
        clean_lat_min, clean_lat_deg, clean_long_min, clean_long_deg = 0,0,0,0
        clean_bearing , clean_ground_speed = 0 , 0
        clean_bearing = self.filter_bearing()
        msg = Odometry(latd_, latm_ , longd_, longm_, bearing_, speed_)
        return msg


    def stationary():
        """Determine if rover is stationary."""
        if self.navstatus == NavState.Off or \
           self.navstatus == NavState.Done:
            return True
        return False

    def turning():
        """Determine if rover is turning."""
        if self.navstatus == NavState.Turn or \
           self.navstatus == NavState.SearchTurn or \
           self.navstatus == NavState.SearchFaceNorth or \
           self.navstatus == NavState.SearchFace120 or \
           self.navstatus == NavState.SearchFace240 or \
           self.navstatus == NavState.SearchFace360 or \
           self.navstatus == NavState.TurnToBall or \
           self.navstatus == NavState.TurnAroundObs or \
           self.navstatus == NavState.SearchTurnAroundObs:
           return True

        return False

    def driving():
        """Determine if rover is driving."""
        if self.navstatus == NavState.Drive or \
           self.navstatus == NavState.SearchDrive or \
           self.navstatus == NavState.DriveToBall or \
           self.navstatus == NavState.DriveAroundObs or \
           self.navstatus == NavState.SearchDriveAroundObs:
           return True

        return False

    def filter_bearing():
        if self.turning() == True:
            if not self.is_turning:
            # Save prev gyro value on first loop of turning.
            # Subtract gyro value from saved previous value to get difference.
            # Fuse mag value with gyro to average it.
                # self.is_turning = True
                self.prev_gyro_x = self.gx
                self.prev_gyro_y = self.gy
                self.prev_gyro_z = self.gz
                self.bearing = (self.bearing+self.mag_bearing)*0.5
            else:
                deltatime = fresh_raw_imu_.time_of_IMU - stale_raw_imu_.time_of_IMU
                self.bearing = self.bearing + deltatime*(self.fresh_raw_imu_.gx - self.stale_raw_imu_.gx) 
                # TODO: not sure what plane is the desired bearing plane;
                self.bearing = (self.bearing + self.mag_bearing)*0.5
                self.prev_gyro_x = self.gx
                self.prev_gyro_y = self.gy
                self.prev_gyro_z = self.gz

        elif self.driving() == True:
            # self.is_turning = False
            pass
        elif self.stationary() == True:
            # self.is_turning = False
            pass
        else:
            print('shitz fukd, check your states homie')


if __name__ == "__main__":
    main()



    