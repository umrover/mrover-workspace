#include <iostream>
#include <cstdint>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/TargetPosition.hpp"
#include "rover_msgs/TargetPositionList.hpp"

using namespace rover_msgs;
using namespace std;

int main() {
    lcm::LCM lcm_;  
    TargetPositionList list;  
    
    while(true) {
        int num_targets;
        cout << "Enter number of tags to send (max 3): ";
        cin >> num_targets;
        list.num_targets = num_targets;
        
        if(list.num_targets > 3 || list.num_targets <= 0) continue;

        for(int i = 0; i < num_targets; i++) {
            cout << "Target " << i << ":\n";
            cout << "X: ";
            cin >> list.target_list[i].x;
            
            cout << "Y: ";
            cin >> list.target_list[i].y;
            
            cout << "Z: ";
            cin >> list.target_list[i].z;

            int target_id;
            cout << "ID: ";
            cin >> target_id;
            list.target_list[i].target_id = target_id;
        }

        lcm_.publish("/target_position_list", &list);
        cout << "List sent!" << endl;
    }
}