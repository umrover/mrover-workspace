//////////////////////////////////////////////////
/////     USER INTERACTION SUPPORT ROUTINES
//////////////////////////////////////////////////

// Controls how fast the target rotates, higher values make the target rotate slower!
kineval.targetRotateSpeed = 100;

kineval.handleUserInput = function user_input() {

    if (keyboard.pressed("shift+left")) {
        console.log('here')
    }

    if ( keyboard.pressed("z") ) {
        camera.position.x += 0.04*(robot.origin.xyz[0]-camera.position.x);
        camera.position.y += 0.04*(robot.origin.xyz[1]+0.5-camera.position.y);
        camera.position.z += 0.04*(robot.origin.xyz[2]-camera.position.z);
    }
    else if ( keyboard.pressed("x") ) {
        camera.position.x -= 0.04*(robot.origin.xyz[0]-camera.position.x);
        camera.position.y -= 0.04*(robot.origin.xyz[1]+0.5-camera.position.y);
        camera.position.z -= 0.04*(robot.origin.xyz[2]-camera.position.z);
    }

    if ( keyboard.pressed("shift+r") ) {
        kineval.params.ik_target.orientation[0] += Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("r") ) {
        kineval.params.ik_target.position[1][0] += 0.01;
    }
    if ( keyboard.pressed("shift+f") ) {
        kineval.params.ik_target.orientation[0] -= Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("f") ) {
        kineval.params.ik_target.position[1][0] -= 0.01;
    }
    if ( keyboard.pressed("shift+a") ) {
        kineval.params.ik_target.orientation[1] += Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("a") ) {
        kineval.params.ik_target.position[0][0] -= 0.02;
    }
    if (keyboard.pressed("shift+d") ) {
        kineval.params.ik_target.orientation[1] -= Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("d") ) {
        kineval.params.ik_target.position[0][0] += 0.02;
    }
    if ( keyboard.pressed("shift+w") ) {
        kineval.params.ik_target.orientation[2] += Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("w") ) {
        kineval.params.ik_target.position[2][0] -= 0.02;
    }
    if (keyboard.pressed("shift+s") ) {
        kineval.params.ik_target.orientation[2] -= Math.PI / kineval.targetRotateSpeed;
    }
    else if ( keyboard.pressed("s") ) {
        kineval.params.ik_target.position[2][0] += 0.02;
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
