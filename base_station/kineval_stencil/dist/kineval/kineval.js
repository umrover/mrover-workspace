
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | core functions

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/



//////////////////////////////////////////////////
/////     KINEVAL OBJECT CONSTRUCTION AND START
//////////////////////////////////////////////////

// create the kineval object/namespace
kineval = {};
console.log("KinEval: Kinematic Evaluator 3");

kineval.zed_matrix = new THREE.Matrix4().makeRotationY(1.5708).setPosition(new THREE.Vector3(0.0381, 0.113, -0.0587));
kineval.zed_matrix_xformed = new THREE.Matrix4().multiply(kineval.zed_matrix);

// function to initialize KinEval and start its animation loop
kineval.start = function kinevalExecute() {

    console.log(" **** >>> kineval.start");
    // KinEval should not do anything until there is a robot and a world loaded
    var x;
    for (x in robot.links) {
        if (typeof links_geom[x] === 'undefined') {
            console.log(JSON.stringify(x))
            console.log("waiting for robot geometries to load");
            //requestAnimationFrame(kineval.start);
            setTimeout(kineval.start,1000);
            return;
        }
    }

    // KinEval uses init() to initialize threejs scene, user input, and robot kinematics
    // STUDENT: you should use my_init() instead
    kineval.init();

    // KinEval uses animate() as the main animation loop maintained by threejs
    // STUDENT: you should use my_animate() instead
    kineval.animate();
}

kineval.init = function init() {

    // initialize robot kinematics
    kineval.initRobot();

    // create kineval params object and set initial values
    kineval.initParameters();

    // initialize threejs and rendering scene
    kineval.initScene();

    // initialize interface parameters and interaction interfaces
    kineval.initInteraction();

    kineval.initlcmbridge();

    // call user's initialization
    my_init();
}

var rgbToHex = function (rgb) { 
    var hex = Number(rgb).toString(16);
    if (hex.length < 2) {
         hex = "0" + hex;
    }
    return hex;
  };

function transpose(arr) {
  return Object.keys(arr[0]).map(function (c) {
    return arr.map(function (r) {
      return r[c];
    });
  });
}

kineval.initlcmbridge = function initlcmbridge() {
    kineval.connections = {
        websocket: false,
        lcm: false,
        motors: false,
        cameras: [false, false, false, false, false, false]
    };


    kineval.publish = function (channel, payload) {
        kineval.lcm.publish(channel, payload);
    };

    kineval.subscribe = function (channel, callbackFn) {
        if( (typeof callbackFn !== "function") || (callbackFn.length !== 1)) {
            console.error("Callback Function is invalid (should take 1 parameter)")
        }
        kineval.lcm.subscribe(channel, callbackFn)
    };

    kineval.lcm = new LCMBridge(
        'ws://localhost:8001',
        // Update WebSocket connection state
        (online) => {
            kineval.lcm.setHomePage()
            kineval.connections.websocket = online
        },
        // Update connection states
        // note: online is an array of bools where first is lcm
        // and everything after is cameras
        (online) => {
            kineval.connections.lcm = online[0],
            kineval.connections.cameras = online.slice(1)
        },
        // Subscribed LCM message received
        async (msg) => {
            if (msg.topic == '/fk_transform') {
                // Parse transform matrix message
                //console.log("got transform")
                var all_links = Object.keys(robot['links'])

                var csys_fix = new THREE.Matrix4().makeRotationX(-1.57079632679)
                var matrix = new THREE.Matrix4()
                matrix.multiplyMatrices(csys_fix, matrix)
                robot.links[robot['base']].xform = matrix
                robot.links[robot.endeffector.name].xform = new THREE.Matrix4()

                for (var link_idx = 1; link_idx < (all_links.length - 1); ++link_idx) {
                    var link_name = all_links[link_idx]
                    var transform = (transpose(msg['message']['transform_' + link_name.slice(0, 1)])).flat()
                    var matrix = new THREE.Matrix4().fromArray(transform)
                    matrix.multiplyMatrices(csys_fix, matrix)
                    robot.links[link_name].xform = matrix
                }

                // to handle hand transform
                var transform = (transpose(msg['message']['transform_f'])).flat()
                var matrix = new THREE.Matrix4().fromArray(transform)
                matrix.multiplyMatrices(csys_fix, matrix)
                robot.links['hand'].xform = matrix

                var all_joints = Object.keys(robot['joints'])

                for (var joint_idx in all_joints) {
                    var joint_name = all_joints[joint_idx]
                    var transform = (transpose(msg['message']['transform_' + joint_name.slice(-1)])).flat();
                    var matrix = new THREE.Matrix4().fromArray(transform)
                    matrix.multiplyMatrices(csys_fix, matrix)
                    robot.joints[joint_name].xform = matrix
                }
            } else if (msg.topic === '/debug_message') {
                
                if (msg['message']['isError']) {
                    console.error(msg['message']['message'])
                } else {
                    console.log(msg['message']['message'])
                }

                if (msg['message']['message'] === 'No IK solution') {
                    target_geom.color = 0xff3300
                    window.alert("No IK solution found. Please try a different configuration.")
                }
                else if (msg['message']['message'] === 'Unsafe Starting Position') {
                    target_geom.color = 0xff3300
                    window.alert("Starting position deemed unsafe. Please adjust in Open Loop.")
                }
                else if (msg['message']['message'].includes("Preview Done")) {

                    // focus window to ensure popup appears
                    while (!document.hasFocus()) {
                        await new Promise(r => setTimeout(r, 200))
                    }

                    // send popup to user
                    shouldExecute = window.confirm("Previewed path. Execute path?");

                    // send lcm accordingly
                    console.log("confirmed path execution: " + shouldExecute)
                    var MotionPreviewMsg = {
                        'type': 'MotionExecute',
                        'execute': shouldExecute,
                    }
                    kineval.publish('/motion_execute', MotionPreviewMsg)
                }
                else if (msg['message']['message'].includes("Encoder Error")) {
                    window.alert(msg['message']['message'])
                }

            }

        },
        // Subscriptions
        [
            {'topic': '/debug_message', 'type': 'DebugMessage'},
            {'topic': '/fk_transform', 'type': 'FKTransform'}
        ]
    )
}

class LCMBridge {
    constructor (url, updateWebsocketState, updateConnectedState,
                 lcmMessage, subscriptions) {
        this.online = false
        this.subscriptions = subscriptions
        this.updateWebsocketState = updateWebsocketState
        this.updateConnectedState = updateConnectedState
        this.lcmMessage = lcmMessage
        this._createWebsocket(url)
        this.callbacks = {}
    }

    _createWebsocket (url) {
        this.ws = new WebSocket(url)

        this.ws.onclose = (event) => {
            this.online = false
            this.updateWebsocketState(this.online)
            this.updateConnectedState([false, false])

            setTimeout(() => {
                this._createWebsocket(url)
            }, 2*1000)
        }

        this.ws.onopen = (event) => {
            this.online = true
            this.updateWebsocketState(this.online)

            // Make subscriptions
            /*for (const sub in this.subscriptions) {
                this._subscribe(sub.topic, sub.type)
            }
            */
            this.subscriptions.forEach(this._subscribe, this)
        }

        this.ws.onmessage = (event) => {
            let event_data = JSON.parse(event.data)
            if (event_data['type'] === 'connection_state') {
                this.updateConnectedState(event_data['state'])
            }
            if (event_data['type'] === 'lcm_message') {
                this.lcmMessage({
                    'topic': event_data['topic'],
                    'message': event_data['message']
                })

                if(this.callbacks[event_data['topic']] !== undefined) {
                    this.callbacks[event_data['topic']].forEach(function(fn){
                        fn(event_data['message'])
                    })
                }
            }
            if (event_data['type'] === 'error_message'){
                console.error(event_data['message']);
            }
        }
    }

    _subscribe ({topic, type}) {
        this.ws.send(JSON.stringify({
            'type': 'lcm_subscribe',
            'topic': topic,
            'lcm_type': type
        }))
    }

    publish (topic, message) {
        if (this.online) {
            this.ws.send(JSON.stringify({
                'type': 'lcm_publish',
                'topic': topic,
                'message': message
            }))
        } else {
            console.error("LCM Bridge not connected")
        }
    }

    subscribe(channel, callbackFn) {
        if(this.callbacks[channel] === undefined){
            this.callbacks[channel] = [callbackFn]
        }else{
            this.callbacks[channel].push(callbackFn)
        }
    }

    setHomePage(){
        if(this.online){
            this.ws.send(JSON.stringify({
                'type':'home_page_set'
            }))
        }
    }
}

//////////////////////////////////////////////////
/////     ANIMATION AND INTERACTION FUNCTIONS
//////////////////////////////////////////////////

kineval.animate = function animate() {

    // THIS IS THE MAIN ANIMATION LOOP

    // note: three.js includes requestAnimationFrame shim
    // alternative to using setInterval for updating in-browser drawing
    // this effectively requests that the animate function be called again for next draw
    // http://learningwebgl.com/blog/?p=3189

    requestAnimationFrame( kineval.animate );

    // call user's animation routine
    my_animate();

    // update camera position and render scene
    kineval.renderScene();
}

kineval.robotDraw = function drawRobot() {

    // robot links
    for (x in robot.links) {

        // KE : properly scope global variable robot_material
        if (kineval.params.display_wireframe)
            robot_material.wireframe = true;
        else
            robot_material.wireframe = false;

        // display links
        simpleApplyMatrix(robot.links[x].geom, robot.links[x].xform);
        robot.links[x].geom.visible = true;
    }

    // robot joints
    for (x in robot.joints) {

        // toggled robot joint display
        if (kineval.params.display_joints) {
            // var tempmat = matrix_mathjs_to_threejs(robot.joints[x].xform);
            simpleApplyMatrix(robot.joints[x].geom, robot.joints[x].xform);
            robot.joints[x].geom.visible = true;
        }
        else
            robot.joints[x].geom.visible = false;

    }

    // toggled display of joint with active control focus
    if (kineval.params.display_joints_active) {
        x = kineval.params.active_joint;
        // var tempmat = robot.joints[x].xform;
        simpleApplyMatrix(robot.joints[x].geom, robot.joints[x].xform);
        robot.joints[x].geom.visible = true;
    }

    if (typeof matrix_multiply !== 'undefined') { // hacked for stencil

    // display robot endeffector
    endeffector_mat = [];
    if (kineval.params.ik_orientation_included) {
        var endeffector_mat = robot.joints[robot.endeffector.frame].xform
        // var b = generate_translation_matrix(robot.endeffector.position[0],robot.endeffector.position[1],robot.endeffector.position[2])
        var ef_pos = new THREE.Vector4().fromArray(robot.endeffector.position)
        var translation_mat = THREE.Matrix4().makeTranslation(ef_pos[0],ef_pos[1],ef_pos[2])
        endeffector_mat.multiply(translation_mat)
        // endeffector_mat = matrix_mathjs_to_threejs(matrix_multiply(robot.joints[robot.endeffector.frame].xform,generate_translation_matrix(robot.endeffector.position[0],robot.endeffector.position[1],robot.endeffector.position[2])));

    } else {
        // endeffector_world represent transform from world to ef frame
        var endeffector_world = robot.joints[robot.endeffector.frame].xform

        // ef_pos is a Vector3 (x,y,z) position of the endeffector
        var ef_pos = new THREE.Vector4().fromArray(robot.endeffector.position)
        // endeffector_world.multiply(robot.endeffector.position)

        // put ef_pos into world frame
        ef_pos.applyMatrix4(endeffector_world)

        // make a Matrix4 from pos. Not sure what this is used for.
        endeffector_mat = new THREE.Matrix4().setPosition(ef_pos)

    }
    //simpleApplyMatrix(endeffector_geom,endeffector_mat);
    simpleApplyMatrix(endeffector_geom,robot.links.hand.xform);
    cart_controller_x_pos.position.setFromMatrixPosition(endeffector_mat)
    cart_controller_y_pos.position.setFromMatrixPosition(endeffector_mat)
    cart_controller_z_pos.position.setFromMatrixPosition(endeffector_mat)
    // cart_controller_x_neg.position.setFromMatrixPosition(endeffector_mat)
    // cart_controller_y_neg.position.setFromMatrixPosition(endeffector_mat)
    // cart_controller_z_neg.position.setFromMatrixPosition(endeffector_mat)
    simpleApplyMatrix(cart_controller_x_pos,robot.links.hand.xform);
    simpleApplyMatrix(cart_controller_y_pos,robot.links.hand.xform);
    simpleApplyMatrix(cart_controller_z_pos,robot.links.hand.xform);

    // display endeffector target
    var three_d_rot = new THREE.Matrix4().makeRotationX(-kineval.params.ik_target.orientation[0])
    three_d_rot.multiply(new THREE.Matrix4().makeRotationY(-kineval.params.ik_target.orientation[2]))
    three_d_rot.multiply(new THREE.Matrix4().makeRotationZ(-kineval.params.ik_target.orientation[1]))

    var trans = new THREE.Matrix4().makeTranslation(kineval.params.ik_target.position[0][0],
                                                kineval.params.ik_target.position[1][0],
                                                kineval.params.ik_target.position[2][0])

    var target_mat = new THREE.Matrix4().multiplyMatrices(trans, three_d_rot)
    simpleApplyMatrix(target_geom,target_mat); // moving AND orienting green cube
    simpleApplyMatrix(cart_controller_x1_pos,target_mat);
    simpleApplyMatrix(cart_controller_y1_pos,target_mat);
    simpleApplyMatrix(cart_controller_z1_pos,target_mat);
    
    // Displays the current target position and orientation 
    posRef.innerHTML = " pos: &nbsp;&nbsp;(" + kineval.params.ik_target.position[0][0].toFixed(2).toString();
    posRef.innerHTML += " " + kineval.params.ik_target.position[1][0].toFixed(2).toString();
    posRef.innerHTML += " " + kineval.params.ik_target.position[2][0].toFixed(2).toString() + ")";
    angRef.innerHTML = " rpy: &nbsp;&nbsp;(" + kineval.params.ik_target.orientation[0].toFixed(3).toString();
    angRef.innerHTML += ", " + kineval.params.ik_target.orientation[1].toFixed(3).toString();
    angRef.innerHTML += ", " + kineval.params.ik_target.orientation[2].toFixed(3).toString() + ")";
    
    // Caluclate and display euler angles:
    // TODO: Convert this to a function (currently duplicated)
    var three_d_rot = new THREE.Matrix4().makeRotationX(-kineval.params.ik_target.orientation[0]);
    three_d_rot.multiply(new THREE.Matrix4().makeRotationY(-kineval.params.ik_target.orientation[2]));
    three_d_rot.multiply(new THREE.Matrix4().makeRotationZ(-kineval.params.ik_target.orientation[1]));
    three_d_rot.multiply(new THREE.Matrix4().makeRotationX(-Math.PI / 2))

    var alpha = Math.atan2(three_d_rot.elements[2], -(three_d_rot.elements[6]));
    var beta = Math.acos(three_d_rot.elements[10]);
    var gamma = Math.atan2(three_d_rot.elements[8], three_d_rot.elements[9]);
    angRef.innerHTML += " <br>euler: (" + alpha.toFixed(3).toString();
    angRef.innerHTML += ", " + beta.toFixed(3).toString();
    angRef.innerHTML += ", " + gamma.toFixed(3).toString() + ")";
    } // hacked for stencil

    endeffector_geom.visible = true;
    target_geom.visible = true;

}

kineval.renderScene = function renderScene() {

    // make sure camera controls (THREE OrbitControls) are looking at the robot base
    camera_controls.target.x = robot.links[robot.base].geom.position.x;
    camera_controls.target.y = robot.links[robot.base].geom.position.y + 0.5;
    camera_controls.target.z = robot.links[robot.base].geom.position.z;

    // threejs rendering update
    renderer.render( scene, camera );
}


//////////////////////////////////////////////////
/////     INITIALIZATION FUNCTION DEFINITONS
//////////////////////////////////////////////////

kineval.initInteraction = function initInteraction() {

    // instantiate threejs keyboard controls, for interactive controls
    keyboard = new THREEx.KeyboardState();

    // create GUI display object and configure
    kineval.initGUIDisplay();

}

kineval.initParameters = function initParameters() {

    // create params object
    kineval.params = {};

    kineval.params.just_starting = true;  // set to true as default, set false once starting forward kinematics project

    // Locked joints aren't allowed to move
    kineval.params.locked_joints = {
        "joint_a": false, 
        "joint_b": false,
        "joint_c": false,
        "joint_d": false,
        "joint_e": false,
        "joint_f": false,
    }

    // initialize the active joint for user control
    kineval.params.active_link = robot.base;
    //kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];

    if (typeof robot.links[kineval.params.active_link].children === 'undefined')
        kineval.params.active_joint = Object.keys(robot.joints)[0]
    else
        kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];

    // initialize inverse kinematics target location
    // KE 3 : ik_target param is redundant as an argument into inverseKinematics
    kineval.params.ik_target = {};
    kineval.params.ik_target.position = [[0],[0.8],[1.0],[1]];
    kineval.params.ik_target.orientation = [Math.PI/6, Math.PI/4, 0];
    kineval.params.ik_orientation_included = false;

    // toggle display of robot links, joints, and axes
    kineval.params.display_joints = false;
    kineval.params.display_joints_axes = false;
    kineval.params.display_wireframe = true;
    kineval.params.display_joints_active = false;
    // kineval.params.display_joints_active_axes = true;

    // apply environment floor with map texture-mapped onto ground plane
    kineval.params.map_filename = url_params.map_filename;
    if (typeof kineval.params.map_filename === 'undefined') kineval.params.display_map = false;
    else kineval.params.display_map = true;

    // simulation_mode is true for not using the real arm
    // simulation_mode is false for using the real arm
    kineval.params.simulation_mode = true;
    kineval.params.use_orientation = false;
}
var cart_lines = []

kineval.initScene = function initScene() {

    // instantiate threejs scene graph
    scene = new THREE.Scene();

    // instantiate threejs camera and set its position in the world
    camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, .01, 10000 );

    // KE 2 : make camera offset from robot vary with interaction, not constant
    camera.position.y = 1;
    camera.position.z = 4;

    var light1 = new THREE.PointLight( 0xffffff, 0.3, 1000 );
    light1.position.set( 50, 50, 50 );
    scene.add( light1 );

    var light2 = new THREE.PointLight( 0xffffff, 0.3, 1000 );
    light2.position.set( 50, 50, -50 );
    scene.add( light2 );

    var light3 = new THREE.PointLight( 0xffffff, 0.3, 1000 );
    light3.position.set( -50, 50, -50 );
    scene.add( light3 );

    var light4 = new THREE.PointLight( 0xffffff, 0.3, 1000 );
    light4.position.set( -50, 50, 50 );
    scene.add( light4 );

    // instantiate threejs renderer and its dimensions
    // THREE r62 renderer = new THREE.WebGLRenderer();
    //renderer = new THREE.WebGLRenderer({antialias: true});
    renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    renderer.setClearColor(0x00234c,1); // blue
    renderer.setClearColor(0xffc90b,1); // maize
    renderer.setClearColor(0xffffff,1); // white
    renderer.setClearColor(0x888888,1); // gray
    renderer.setSize( window.innerWidth, window.innerHeight );

    // attach threejs renderer to DOM
    document.body.appendChild( renderer.domElement );

    // instantiate threejs camera controls
    camera_controls = new THREE.OrbitControls( camera, renderer.domElement );
    //camera_controls.addEventListener( 'change', renderer );

    // create world floor
    // KE T creates error : "TypeError: n.x is undefined"
    // THREE r62 var mapMaterial = new THREE.MeshBasicMaterial( { map: kineval.params.map_texture, transparent: true, opacity: 0.2 } );
    var mapMaterial = new THREE.MeshBasicMaterial( { color: 0x00234c , transparent: true, opacity: 0.5 } );
    var mapGeometry = new THREE.PlaneGeometry(100, 100, 1, 1);
    map = new THREE.Mesh(mapGeometry, mapMaterial);
    map.doubleSided = true;
    //map.receiveShadow = true; // KE T: recheck to make sure this works
    map.rotateOnAxis({x:1,y:0,z:0},-Math.PI/2),
    scene.add(map);


    // create grid on floor
    // (73) gridHelper = new THREE.GridHelper( 50, 5, 0xffc90b, 0x00234c);
    // (73) gridHelper.setColors(0xffc90b,0x00234c);
    gridHelper = new THREE.GridHelper( 100, 20, 0xffc90b, 0x00234c);
    gridHelper.translateOnAxis(new THREE.Vector3(0,1,0),0.02);
    gridHelper.material.transparent = true;
    gridHelper.material.opacity = 0.2;
    scene.add( gridHelper );

    // create geometry for endeffector and Cartesian target indicators
    var temp_geom = new THREE.CubeGeometry(0.1, 0.1, 0.1);
    var temp_material = new THREE.MeshBasicMaterial( {color: 0x0088ff} ) // blue cube
    endeffector_geom = new THREE.Mesh(temp_geom, temp_material); // comment this for coolness
    scene.add(endeffector_geom);
    endeffector_geom.visible = false;

    // Add target to look like hand (see mrover_arm_urdf.js)
    target_geom = links_geom["target"];
    scene.add(target_geom);
    target_geom.visible = false;

    // create threejs geometries for robot links
    kineval.initRobotLinksGeoms();

    // create threejs geometries for robot joints
    kineval.initRobotJointsGeoms();

    // create threejs geometries for robot planning scene
    kineval.initWorldPlanningScene();

    // create geometry for cartesian controller
    // xyz axis indicators
    var red = new THREE.LineBasicMaterial({color: 0xff0000});
    var grn = new THREE.LineBasicMaterial({color: 0x00ff00});
    var blu = new THREE.LineBasicMaterial({color: 0x0000ff});
    var cart_controller_x_pos_geom = new THREE.Geometry();
    cart_controller_x_pos_geom.vertices.push(
        new THREE.Vector3(0,0,0),
        new THREE.Vector3(0.3,0,0)
    );
    cart_controller_x_pos = new THREE.Line(cart_controller_x_pos_geom, red)
    scene.add(cart_controller_x_pos);
    cart_lines.push(cart_controller_x_pos);
    cart_controller_x_pos.visible = true;

    cart_controller_x1_pos = new THREE.Line(cart_controller_x_pos_geom, red)
    scene.add(cart_controller_x1_pos);
    cart_lines.push(cart_controller_x1_pos);
    cart_controller_x1_pos.visible = true;

    var cart_controller_y_pos_geom = new THREE.Geometry();
    cart_controller_y_pos_geom.vertices.push(
        new THREE.Vector3(0,0,0),
        new THREE.Vector3(0,0.3,0)
    );
    cart_controller_y_pos = new THREE.Line(cart_controller_y_pos_geom, grn)
    scene.add(cart_controller_y_pos);
    cart_lines.push(cart_controller_y_pos);
    cart_controller_y_pos.visible = true;

    cart_controller_y1_pos = new THREE.Line(cart_controller_y_pos_geom, grn)
    scene.add(cart_controller_y1_pos);
    cart_lines.push(cart_controller_y1_pos);
    cart_controller_y1_pos.visible = true;

    var cart_controller_z_pos_geom = new THREE.Geometry();
    cart_controller_z_pos_geom.vertices.push(
        new THREE.Vector3(0,0,0),
        new THREE.Vector3(0,0,0.3)
    );
    cart_controller_z_pos = new THREE.Line(cart_controller_z_pos_geom, blu)
    scene.add(cart_controller_z_pos);
    cart_lines.push(cart_controller_z_pos);
    cart_controller_z_pos.visible = true;

    cart_controller_z1_pos = new THREE.Line(cart_controller_z_pos_geom, blu)
    scene.add(cart_controller_z1_pos);
    cart_lines.push(cart_controller_z1_pos);
    cart_controller_z1_pos.visible = true;

    // var cart_controller_x_neg_geom = new THREE.Geometry();
    // cart_controller_x_neg_geom.vertices.push(
    //     new THREE.Vector3(0,0,0),
    //     new THREE.Vector3(-0.3,0,0)
    // );
    // cart_controller_x_neg = new THREE.Line(cart_controller_x_neg_geom, red)
    // scene.add(cart_controller_x_neg);
    // cart_lines.push(cart_controller_x_neg);
    // cart_controller_x_neg.visible = true;

    // var cart_controller_y_neg_geom = new THREE.Geometry();
    // cart_controller_y_neg_geom.vertices.push(
    //     new THREE.Vector3(0,0,0),
    //     new THREE.Vector3(0,-0.3,0)
    // );
    // cart_controller_y_neg = new THREE.Line(cart_controller_y_neg_geom, grn)
    // scene.add(cart_controller_x_neg);
    // cart_lines.push(cart_controller_y_neg);
    // cart_controller_y_neg.visible = true;

    // var cart_controller_z_neg_geom = new THREE.Geometry();
    // cart_controller_z_neg_geom.vertices.push(
    //     new THREE.Vector3(0,0,0),
    //     new THREE.Vector3(0,0,-0.3)
    // );
    // cart_controller_z_neg = new THREE.Line(cart_controller_z_neg_geom, blu)
    // scene.add(cart_controller_z_neg);
    // cart_lines.push(cart_controller_z_neg);
    // cart_controller_z_neg.visible = true;

    var geometry = new THREE.SphereBufferGeometry( 0.02 );
    var material = new THREE.MeshBasicMaterial( { color: 0xffffff } );
    sphereInter = new THREE.Mesh( geometry, material );
    sphereInter.visible = false;
    scene.add( sphereInter );
}

kineval.initGUIDisplay = function initGUIDisplay () {

    var primary_gui = new dat.GUI();
    // console.log(kineval.params);

    var primary_display = {};
    primary_display.send_target_orientation = function() {
        
        // rpy to euler angles conversion
        var three_d_rot = new THREE.Matrix4().makeRotationX(kineval.params.ik_target.orientation[0])
        three_d_rot.multiply(new THREE.Matrix4().makeRotationY(kineval.params.ik_target.orientation[2]))
        three_d_rot.multiply(new THREE.Matrix4().makeRotationZ(kineval.params.ik_target.orientation[1]))
        three_d_rot.multiply(new THREE.Matrix4().makeRotationX(-Math.PI / 2))

        var alph = Math.atan2(three_d_rot.elements[2], -(three_d_rot.elements[6]));
        var bet = Math.acos(three_d_rot.elements[10]);
        var gam = Math.atan2(three_d_rot.elements[8], three_d_rot.elements[9]);

        var TargetOrientationMsg =  {
            'type': 'TargetOrientation',
            'x': kineval.params.ik_target.position[0][0],
            'y': -1 * kineval.params.ik_target.position[2][0],
            'z': kineval.params.ik_target.position[1][0],
            'alpha': alph,
            'beta': bet,
            'gamma': gam,
        }

        kineval.publish('/target_orientation', TargetOrientationMsg)
        console.log("sent point")
        console.log(kineval.params.use_orientation)
    }

    // 1. send point
    // 2. preview 
    // 3. execute 
    primary_gui.add(primary_display, 'send_target_orientation');
    
}

kineval.initRobotLinksGeoms = function initRobotLinksGeoms() {

    // KE 2 : put robot_material into correct object (fixed below?)
    // KE ! : this may need to be moved back into link for loop
    robot_material = new THREE.MeshLambertMaterial( { color: 0x00234c, transparent: true, opacity: 0.9 } );
    //robot_material = new THREE.MeshLambertMaterial( { color: 0x00234c, transparent: true, opacity: 0.9, wireframe: true } );

    // create a threejs mesh for link of the robot and add it to scene
    for (x in robot.links) {


        // create threejs mesh for link
        // handle conversion to ROS coordinate convention
        // KE 2 : create global color constants
        if (typeof robot.links_geom_imported === "undefined")
            robot.links[x].geom = new THREE.Mesh( links_geom[x], robot_material);
        else if (!robot.links_geom_imported)
            robot.links[x].geom = new THREE.Mesh( links_geom[x], robot_material);
        else
            robot.links[x].geom = links_geom[x];
        robot.links[x].geom.name = "robot_link_"+x;

        // add to threejs mesh to scene in world frame
        // KE : defer this add until child nodes are added to the geom
        //scene.add(robot.links[x].geom);

        // remove any transform from the threejs geometry for bbox calculation
        robot.links[x].geom.setRotationFromQuaternion(new THREE.Quaternion(0,0,0,1));

        // add to threejs mesh to scene in world frame
        scene.add(robot.links[x].geom);
    }

}

kineval.initRobotJointsGeoms = function initRobotJointsGeoms() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs
    // NOTE: simpleApplyMatrix can be used to set threejs transform for a rendered object

    var x,tempmat;

    // create threejs geometry for joint origin
    material = new THREE.MeshBasicMaterial( { color: 0xff0000, wireframe: true } );
    invisible_geom = new THREE.CubeGeometry( 0.01, 0.01, 0.01 );

    // create threejs geometry for joint
    temp_material = new THREE.MeshBasicMaterial( { color: 0xff0000, wireframe: true } );

    joints_geom = new THREE.CubeGeometry( 0.01, 0.01, 0.01 );

    // KE 2 : create global color constants
    // KE 2 : fix lighting, use ambient until fixed
    joint_material = new THREE.MeshBasicMaterial( {color: 0xffc90b} );
    //joint_material = new THREE.MeshLambertMaterial( {color: 0xffc90b} );
    //joint_material = new THREE.MeshLambertMaterial( {color: 0xff0000} );

    for (x in robot.joints) {

        // create threejs meshes for joints
        robot.joints[x].origin.geom = new THREE.Mesh( invisible_geom, material );
        robot.joints[x].geom = new THREE.Mesh( joints_geom, temp_material );


        // Note: kinematics are maintained independently from threejs scene graph
        // add joint geometry to threejs scene graph, added SG node transforms cylinder geometry
        // handle conversion to ROS coordinate convention
        if (typeof robot.links_geom_imported === "undefined")
           var joint_geom = new THREE.CylinderGeometry( 0.2, 0.2, 0.2, 20, 3, false );  // cylinder axis aligns with along y-axis in object space
            //var joint_geom = new THREE.CylinderGeometry( 0.12, 0.12, 0.02, 20, 3, false );  // cylinder axis aligns with along y-axis in object space
        else if (robot.links_geom_imported)
            var joint_geom = new THREE.CylinderGeometry( 0.12, 0.12, 0.02, 20, 3, false );  // cylinder axis aligns with along y-axis in object space
        else
           var joint_geom = new THREE.CylinderGeometry( 0.2, 0.2, 0.2, 20, 3, false );  // cylinder axis aligns with along y-axis in object space

        robot.joints[x].display_geom = new THREE.Mesh(joint_geom, joint_material);

        // STENCIL: update vector_normalize for joint cylinder placement
        // if joint axis not aligned with y-axis, rotate 3js cylinder axis to align with y
        if (typeof vector_cross !== 'undefined')
        if (!((robot.joints[x].axis[0] == 0) && (robot.joints[x].axis[2] == 0))) {
            var temp3axis = new THREE.Vector3().fromArray(robot.joints[x].axis)
            var other = new THREE.Vector3(0, -1, 0)
            temp3axis.cross(other).normalize()

            // baked in dot product given cylinder axis is normal along y-axis
            var tempangle = Math.acos(robot.joints[x].axis[1]);
            robot.joints[x].display_geom.rotateOnAxis(temp3axis,tempangle);
        }
        scene.add(robot.joints[x].geom);
        robot.joints[x].geom.add(robot.joints[x].display_geom);

    }
}

kineval.initWorldPlanningScene = function initWorldPlanningScene() {
    // currently just sets rendering geometries
    // world defined by robot_boundary and robot_obstacles objects in separate js include

    // set rendering geometries of world boundary
    temp_material = new THREE.MeshLambertMaterial( { color: 0xaf8c73, transparent: true, opacity: 0.6} );
    //temp_material = new THREE.MeshLambertMaterial( { color: 0xaf8c73, transparent: true, opacity: 0.6, wireframe: true} );

    temp_geom = new THREE.CubeGeometry(robot_boundary[1][0]-robot_boundary[0][0],0.2,0.2);
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = (robot_boundary[1][0]+robot_boundary[0][0])/2;
    temp_mesh.position.y = 0;
    temp_mesh.position.z = robot_boundary[0][2];
    scene.add(temp_mesh);

    temp_geom = new THREE.CubeGeometry(robot_boundary[1][0]-robot_boundary[0][0],0.2,0.2);
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = (robot_boundary[1][0]+robot_boundary[0][0])/2;
    temp_mesh.position.y = 0;
    temp_mesh.position.z = robot_boundary[1][2];
    scene.add(temp_mesh);

    temp_geom = new THREE.CubeGeometry(0.2,0.2,robot_boundary[1][2]-robot_boundary[0][2]);
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = robot_boundary[0][0];
    temp_mesh.position.y = 0;
    temp_mesh.position.z = (robot_boundary[1][2]+robot_boundary[0][2])/2;
    scene.add(temp_mesh);

    temp_geom = new THREE.CubeGeometry(0.2,0.2,robot_boundary[1][2]-robot_boundary[0][2]);
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = robot_boundary[1][0];
    temp_mesh.position.y = 0;
    temp_mesh.position.z = (robot_boundary[1][2]+robot_boundary[0][2])/2;
    scene.add(temp_mesh);
}

//////////////////////////////////////////////////
/////     FILE LOADING FUNCTIONS
//////////////////////////////////////////////////

kineval.loadJSFile = function loadJSFile(filename,kineval_object) {

    // load JavaScript file dynamically from filename, and (optionally) assign to recognized field of kineval object
    // WARNING: execution of the kineval main loop must wait until the specified file is loaded.  For the browser, this is accomplished by having kineval.start() called within the window.onload() function of the executing HTML document

    // create HTML script element and set its type and source file
    robotDefinitionElement = document.createElement('script');
    robotDefinitionElement.setAttribute("type","text/javascript");
    robotDefinitionElement.setAttribute("src",filename);

    // assuming this element is created, append it to the head of the referring HTML document
    if (typeof robotDefinitionElement !== 'undefined') {
        document.getElementsByTagName("head")[0].appendChild(robotDefinitionElement);
        kineval[kineval_object+"_file"] = filename;
    }
    else
    {
        console.warn("kineval: "+filename+" not loaded");
    }

    if (kineval_object!=="robot" && kineval_object!=="world" && kineval_object!=="floor")
        console.warn("kineval: JS file loaded, object type "+kineval_object+" not recognized");

}

function loadJSON(callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType("application/json");

    // open json preset file
    xobj.open('GET', './mrover_arm_presets.json', true);

    // once file is loaded
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == "200") {
            console.log('Loaded presets json');
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            callback(xobj.responseText);
        }
        else {
            console.log('Could not load presets json');
        }
    };
    xobj.send(null);  
}

// create menu for presets
function presets_init(gui) {

    // pass callback to loadJSON to be called when it gets data from json file
    loadJSON(function(response) {
        // Parse JSON string into object
        actual_JSON = JSON.parse(response);
        console.log("presets json =" + JSON.stringify(actual_JSON));

        var size = Object.keys(actual_JSON).length;
        var presetToggles = { };
        for(i = 0; i < size; i++) {
            presetToggles[Object.keys(actual_JSON)[i]] = false;
            gui.add(presetToggles, Object.keys(actual_JSON)[i]);
        }

        var submit = new presetAngles();
        submit.presetToggles = presetToggles;
        submit.actual_JSON = actual_JSON;
        submit.size = size;
        gui.add(submit, "submit");
    });
}
