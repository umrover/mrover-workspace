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
            }
            if ((keyboard.pressed("b"))&&(kineval.motion_plan_traversal_index>0)) {
                kineval.motion_plan_traversal_index--;
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
        robot.joints[kineval.params.active_joint].control += 0.05;  // add motion increment
    }
    else if ( keyboard.pressed("i") ) {
        robot.joints[kineval.params.active_joint].control += -0.05;  // add motion increment
    }
    if ( keyboard.pressed("shift+r") ) {
        kineval.params.ik_target.orientation[0] += Math.PI / 24;
    }
    else if ( keyboard.pressed("r") ) {
        kineval.params.ik_target.position[1][0] += 0.01;
    }
    if ( keyboard.pressed("shift+f") ) {
        kineval.params.ik_target.orientation[0] -= Math.PI / 24;
    }
    else if ( keyboard.pressed("f") ) {
        kineval.params.ik_target.position[1][0] -= 0.01;
    }
    if ( keyboard.pressed("shift+a") ) {
        kineval.params.ik_target.orientation[1] += Math.PI / 24;
    }
    else if ( keyboard.pressed("a") ) {
        kineval.params.ik_target.position[0][0] -= 0.02;
    }
    if (keyboard.pressed("shift+d") ) {
        kineval.params.ik_target.orientation[1] -= Math.PI / 24;
    }
    else if ( keyboard.pressed("d") ) {
        kineval.params.ik_target.position[0][0] += 0.02;
    }
    if ( keyboard.pressed("shift+w") ) {
        kineval.params.ik_target.orientation[2] += Math.PI / 24;
    }
    else if ( keyboard.pressed("w") ) {
        kineval.params.ik_target.position[2][0] -= 0.02;
    }
    if (keyboard.pressed("shift+s") ) {
        kineval.params.ik_target.orientation[2] -= Math.PI / 24;
    }
    else if ( keyboard.pressed("s") ) {
        kineval.params.ik_target.position[2][0] += 0.02;
    }
    if (keyboard.pressed("g")) {
        console.log(JSON.stringify(kineval.setpoints));
    }

    if (keyboard.pressed("v")) {
        kineval.displayHelp();
    }
}

kineval.displayHelp = function display_help () {
        // textbar.innerHTML = "kineval user interface commands"
        //     + "<br>mouse: rotate camera about robot base "
        //     + "<br>z/x : camera zoom with respect to base "
        //     + "<br>t : toggle starting point mode "
        //     + "<br>w/s a/d q/e : move base along forward/turning/strafe direction"
        //     + "<br>j/k/l : focus active joint to child/parent/sibling "
        //     + "<br>u/i : control active joint"
        //     + "<br>c : execute clock tick controller "
        //     + "<br>o : control robot arm to current setpoint target "
        //     + "<br>0 : control arm to zero pose "
        //     + "<br>Shift+[1-9] : assign current pose to a pose setpoint"
        //     + "<br>[1-9] : assign a pose setpoint to current setpoint target"
        //     + "<br>g : print pose setpoints to console "
        //     + "<br>p : iterate inverse kinematics motion "
        //     + "<br>r/f : move inverse kinematics target up/down"
        //     + "<br>m : invoke motion planner "
        //     + "<br>n/b : show next/previous pose in motion plan "
        //     + "<br>h : toggle gui command widget ";
        //     + "<br>v : print commands to screen ";
}


kineval.toggleStartpointMode = function toggle_startpoint_mode() {
    kineval.params.just_starting = !kineval.params.just_starting;
    kineval.publish_joint_angles()
}


kineval.changeActiveLinkDown = function change_active_link_down() {
    if (typeof robot.links[robot.joints[kineval.params.active_joint].child].children !== 'undefined') {
        kineval.params.active_link = robot.joints[kineval.params.active_joint].child;
        kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];
    }
}

kineval.changeActiveLinkUp = function change_active_link_up() {
    if (kineval.params.active_link !== robot.base) {
        kineval.params.active_joint = robot.links[kineval.params.active_link].parent;
        kineval.params.active_link = robot.joints[kineval.params.active_joint].parent;
    }
}

kineval.changeActiveLinkNext = function change_active_joint_next() {

    kineval.params.active_joint = robot.links[kineval.params.active_link].children[(robot.links[kineval.params.active_link].children.indexOf(kineval.params.active_joint)+1) % robot.links[kineval.params.active_link].children.length];

}

kineval.changeActiveLinkPrevious = function change_active_joint_previous() {

    kineval.params.active_joint = robot.links[kineval.params.active_link].children[(robot.links[kineval.params.active_link].children.length + robot.links[kineval.params.active_link].children.indexOf(kineval.params.active_joint)-1) % robot.links[kineval.params.active_link].children.length];

}


