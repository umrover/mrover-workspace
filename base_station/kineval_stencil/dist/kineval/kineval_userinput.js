//////////////////////////////////////////////////
/////     USER INTERACTION SUPPORT ROUTINES
//////////////////////////////////////////////////

kineval.initKeyEvents = function init_keyboard_events() {
    document.addEventListener('keydown', function(e) {
        if (e.repeat) {
            return
        }
        // console.log(e); // uncomment this line for key messages
        kineval.handleKeydown(e.keyCode); }, true);
}

kineval.handleKeydown = function handle_keydown(keycode) {
    //console.log("handle_keydown: "+keycode);
    switch (keycode) { // h:72 j:74 k:75 l:76
    case 74: // j
        kineval.changeActiveLinkDown();
        break;
    case 75: // k
        kineval.changeActiveLinkUp();
        break;
    case 76: // l
        kineval.changeActiveLinkNext();
        break;
    case 72: // h
        kineval.changeActiveLinkPrevious();
        break;
    case 84: // t
        kineval.toggleStartpointMode();
        break;
    case 37: // arrow down
        // rosCmdVel.publish(rosTwistLft);
        console.log('trying to move left');
        break;
    case 38: // arrow up
        // rosCmdVel.publish(rosTwistFwd);
        console.log('trying to move forward');
        break;
    case 39: // arrow up
        // rosCmdVel.publish(rosTwistRht);
        console.log('trying to move right');
        break;
    case 40: // arrow left
        // rosCmdVel.publish(rosTwistBwd);
        console.log('trying to move backward');
        break;
    case 13: // enter
        // rosManip.publish(rosManipGrasp);
        console.log('trying to grasp');
        break;
    case 49: // send IK target with '1' key
        var TargetOrientationMsg =  {
            'type': 'TargetOrientation',
            'x': kineval.params.ik_target.position[0][0],
            'y': -1 * kineval.params.ik_target.position[2][0],
            'z': kineval.params.ik_target.position[1][0],  
        }

        kineval.publish('/target_orientation', TargetOrientationMsg)
        break;
    case 50: // send preview command with '2' key
        var MotionExecuteMsg = {
            'type': 'MotionExecute',
            'preview': true,
        }
        console.log('hello')
        // textbar.innerHTML = "execute preview";
        kineval.publish('/motion_execute', MotionExecuteMsg)
        break;
    case 51: // send execute controller command with '3'
        var MotionExecuteMsg = { 
            'type': 'MotionExecute',
            'preview': false,
        }
        // textbar.innerHTML = "execute motor controls";
        kineval.publish('/motion_execute', MotionExecuteMsg)
        break;
    }
}

kineval.handleUserInput = function user_input() {

    if (keyboard.pressed("shift+left")) {
        console.log('here')
    }

    if ( keyboard.pressed("z") ) {
        camera.position.x += 0.1*(robot.origin.xyz[0]-camera.position.x);
        camera.position.y += 0.1*(robot.origin.xyz[1]+0.5-camera.position.y);
        camera.position.z += 0.1*(robot.origin.xyz[2]-camera.position.z);
    }
    else if ( keyboard.pressed("x") ) {
        camera.position.x -= 0.1*(robot.origin.xyz[0]-camera.position.x);
        camera.position.y -= 0.1*(robot.origin.xyz[1]+0.5-camera.position.y);
        camera.position.z -= 0.1*(robot.origin.xyz[2]-camera.position.z);
    }

    // request generation of motion plan
    if ( keyboard.pressed("m") )
        kineval.params.update_motion_plan = true;

    // traverse generated motion plan
    if ( keyboard.pressed("n") |  keyboard.pressed("b")) {

        kineval.params.update_motion_plan_traversal = true;

        if (kineval.motion_plan.length > 0) {

            // increment index
            if ((keyboard.pressed("n"))&&(kineval.motion_plan_traversal_index<kineval.motion_plan.length-1)) {
                kineval.motion_plan_traversal_index++;
                textbar.innerHTML = "moved robot forward along planned motion trajectory";
            }
            if ((keyboard.pressed("b"))&&(kineval.motion_plan_traversal_index>0)) {
                kineval.motion_plan_traversal_index--;
                textbar.innerHTML = "moved robot backward along planned motion trajectory";
            }
        }
    }

    // execute inverse kinematics
    if ( keyboard.pressed("p") )
        kineval.params.update_ik = true;

    // execute PID controller to setpoint
    if ( keyboard.pressed("o") ) {
        kineval.params.update_pd = true;
        kineval.params.update_pd_clock = false;
        kineval.params.update_pd_dance = false;
    }

    // execute PID controller to clock
    if ( keyboard.pressed("c") ) {
        kineval.params.update_pd = true;
        kineval.params.update_pd_clock = true;
    }

    // incrment/decrement angle of active joint
    if ( keyboard.pressed("u") ) {
        textbar.innerHTML = "active joint is moving in positive direction";
        robot.joints[kineval.params.active_joint].control += 0.05;  // add motion increment
    }
    else if ( keyboard.pressed("i") ) {
        textbar.innerHTML = "active joint is moving in negative direction";
        robot.joints[kineval.params.active_joint].control += -0.05;  // add motion increment
    }

    // KE : this needs to be stencilized
    if ( keyboard.pressed("q") ) {  // strafe
        textbar.innerHTML = "moving base left";
        //robot.origin.xyz[0] += 0.1; // simple but ineffective: not aligned with robot

        robot.control.xyz[2] += 0.1 * (robot_lateral[2][0]-robot.origin.xyz[2]);
        robot.control.xyz[0] += 0.1 * (robot_lateral[0][0]-robot.origin.xyz[0]);
    }
    if ( keyboard.pressed("e") ) {  // strafe
        textbar.innerHTML = "moving base right";
        // robot.origin.xyz[0] -= 0.1; // simple but ineffective: not aligned with robot

        robot.control.xyz[2] += -0.1 * (robot_lateral[2][0]-robot.origin.xyz[2]);
        robot.control.xyz[0] += -0.1 * (robot_lateral[0][0]-robot.origin.xyz[0]);
    }

    if (keyboard.pressed("1")) {
        textbar.innerHTML = "Sent Target point, solving Ik"
    }

    if ( keyboard.pressed("shift+r") ) {
        kineval.params.ik_target.orientation[0] += Math.PI / 24;
        textbar.innerHTML = "ik orient: " + kineval.params.ik_target.orientation[0];
    }
    else if ( keyboard.pressed("r") ) {
        textbar.innerHTML = "moving IK target up";
        kineval.params.ik_target.position[1][0] += 0.01;
    }
    if ( keyboard.pressed("shift+f") ) {
        kineval.params.ik_target.orientation[0] -= Math.PI / 24;
        textbar.innerHTML = "ik orient: " + kineval.params.ik_target.orientation[0];
    }
    else if ( keyboard.pressed("f") ) {
        textbar.innerHTML = "moving IK target down";
        kineval.params.ik_target.position[1][0] -= 0.01;
    }
    if ( keyboard.pressed("shift+a") ) {
        kineval.params.ik_target.orientation[1] += Math.PI / 24;
    }
    else if ( keyboard.pressed("a") ) {
        textbar.innerHTML = "moving IK target left";
        kineval.params.ik_target.position[0][0] -= 0.02;
    }
    if (keyboard.pressed("shift+d") ) {
        kineval.params.ik_target.orientation[1] -= Math.PI / 24;
    }
    else if ( keyboard.pressed("d") ) {
        textbar.innerHTML = "moving IK target right";
        kineval.params.ik_target.position[0][0] += 0.02;
    }
    if ( keyboard.pressed("shift+w") ) {
        kineval.params.ik_target.orientation[2] += Math.PI / 24;
    }
    else if ( keyboard.pressed("w") ) {
        textbar.innerHTML = "moving IK target forward";
        kineval.params.ik_target.position[2][0] -= 0.02;
    }
    if (keyboard.pressed("shift+s") ) {
        kineval.params.ik_target.orientation[2] -= Math.PI / 24;
    }
    else if ( keyboard.pressed("s") ) {
        textbar.innerHTML = "moving IK target backward";
        kineval.params.ik_target.position[2][0] += 0.02;
    }
    // if ( keyboard.pressed("1") ) {
    //     textbar.innerHTML = "publish target point";
    //     var TargetPointMsg =  {
    //         'type': 'TargetPoint',
    //         'x': kineval.params.ik_target.position[0][0],
    //         'y': -1 * kineval.params.ik_target.position[2][0],
    //         'z': kineval.params.ik_target.position[1][0],
    //     }

    //     kineval.publish('/target_point', TargetPointMsg)
    // }
    if (keyboard.pressed("g")) {
        textbar.innerHTML = "pose setpoints printed to console";
        console.log(JSON.stringify(kineval.setpoints));
    }

    if (keyboard.pressed("v")) {
        kineval.displayHelp();
    }
}

kineval.displayHelp = function display_help () {
        textbar.innerHTML = "kineval user interface commands"
            + "<br>mouse: rotate camera about robot base "
            + "<br>z/x : camera zoom with respect to base "
            + "<br>t : toggle starting point mode "
            + "<br>w/s a/d q/e : move base along forward/turning/strafe direction"
            + "<br>j/k/l : focus active joint to child/parent/sibling "
            + "<br>u/i : control active joint"
            + "<br>c : execute clock tick controller "
            + "<br>o : control robot arm to current setpoint target "
            + "<br>0 : control arm to zero pose "
            + "<br>Shift+[1-9] : assign current pose to a pose setpoint"
            + "<br>[1-9] : assign a pose setpoint to current setpoint target"
            + "<br>g : print pose setpoints to console "
            + "<br>p : iterate inverse kinematics motion "
            + "<br>r/f : move inverse kinematics target up/down"
            + "<br>m : invoke motion planner "
            + "<br>n/b : show next/previous pose in motion plan "
            + "<br>h : toggle gui command widget ";
            + "<br>v : print commands to screen ";
}


kineval.toggleStartpointMode = function toggle_startpoint_mode() {
    textbar.innerHTML = "toggled startpoint mode";
    kineval.params.just_starting = !kineval.params.just_starting;
    kineval.publish_joint_angles()
}


kineval.changeActiveLinkDown = function change_active_link_down() {
    if (typeof robot.links[robot.joints[kineval.params.active_joint].child].children !== 'undefined') {
        kineval.params.active_link = robot.joints[kineval.params.active_joint].child;
        kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];
        textbar.innerHTML = kineval.params.active_joint + " is now the active joint";
    }
}

kineval.changeActiveLinkUp = function change_active_link_up() {
    if (kineval.params.active_link !== robot.base) {
        kineval.params.active_joint = robot.links[kineval.params.active_link].parent;
        kineval.params.active_link = robot.joints[kineval.params.active_joint].parent;
        textbar.innerHTML = kineval.params.active_joint + " is now the active joint";
    }
}

kineval.changeActiveLinkNext = function change_active_joint_next() {

    kineval.params.active_joint = robot.links[kineval.params.active_link].children[(robot.links[kineval.params.active_link].children.indexOf(kineval.params.active_joint)+1) % robot.links[kineval.params.active_link].children.length];

    textbar.innerHTML = kineval.params.active_joint + " is now the active joint";
}

kineval.changeActiveLinkPrevious = function change_active_joint_previous() {

    kineval.params.active_joint = robot.links[kineval.params.active_link].children[(robot.links[kineval.params.active_link].children.length + robot.links[kineval.params.active_link].children.indexOf(kineval.params.active_joint)-1) % robot.links[kineval.params.active_link].children.length];

    textbar.innerHTML = kineval.params.active_joint + " is now the active joint";
}


