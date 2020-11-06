
#include "rover_msgs/GimbalCmd"

class Gimbal{
    private:
        double yaw_power, pitch_power;
        double cur_yaw, cur_pitch;
        double target_yaw, target_pitch;
        const double MAX_YAW, MIN_YAW, MAX_PITCH, MIN_PITCH;
    public:
        //sets the target yaw of the gimbal
        void setTargetYaw(double target);
        //sets the target pitch of the gimbal
        void setTargetPitch(double target);
        //returns the LCM message to be sent
        void publishControlSignal();
};