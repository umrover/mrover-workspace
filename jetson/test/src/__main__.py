import lcm
from rover_msgs import DriveVelCmd

lcm_ = lcm.LCM()

speed = [0.5, -0.5, 0]

def main():
    it = 2
    jt = 0

    while(1):
        state = it % 3 
        vel_cmd = DriveVelCmd()

        vel_cmd.left = speed[state]
        vel_cmd.right = speed[state]
        
        lcm_.publish("/drive_vel_cmd", vel_cmd.encode())

        jt += 1

        if (jt == 100000):
            jt = 0
            it += 1
            print("state: " + str(it % 3) + " speed: " + str(speed[it % 3]))
        

if __name__ == "__main__":
    main()


        


