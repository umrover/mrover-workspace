
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
        (msg) => {
            if (msg.topic == '/arm_position') {
                var all_joints = Object.keys(robot.joints).slice(0, Object.keys(robot.joints).length - 1)
                for (var joint_idx in all_joints) {
                    var joint_name = all_joints[joint_idx]
                    var axis = robot.joints[joint_name]['axis']
                    robot.joints[joint_name].angle = msg['message'][joint_name]
                }
                // send confirmation of angles so we know to send angle
                msg['message']['type'] = 'ArmPosition'
                // send confirmation back
                kineval.publish('/confirmation', msg['message'])
            }
            else if (msg.topic == '/fk_transform') {
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
            } else if (msg.topic === '/debugMessage') {
                
                if (msg['message']['isError']) {
                    console.error(msg['message']['message'])
                } else {
                    console.log(msg['message']['message'])
                }
                if (msg['message']['message'] === 'Solved IK') {
                    textbar.innerHTML = 'Solved IK. Planning Path'
                    target_geom.color = 0x00ff00
                }
                else if (msg['message']['message'] === 'No IK solution') {
                    textbar.innerHTML = 'No Ik solution, using closest Config' 
                    target_geom.color = 0xff3300
                }
                else if (msg['message']['message'].includes("Planned path")) {
                    textbar.innerHTML = msg['message']['message']
                    shouldPreview = window.confirm("Planned Path. View Path?");
                    if (shouldPreview) {
                        var MotionPreviewMsg = {
                            'type': 'MotionExecute',
                            'preview': true,
                        }
                        console.log('Previewing plan')
                        // textbar.innerHTML = "execute preview";
                        kineval.publish('/motion_execute', MotionPreviewMsg)
                    }
                }
                else if (msg['message']['message'].includes("Preview")) {
                    shouldExecute = window.confirm("Previewed path. Execute Path?");
                    if (shouldExecute) {
                        var MotionPreviewMsg = {
                            'type': 'MotionExecute',
                            'preview': false,
                        }
                        // textbar.innerHTML = "execute preview";
                        kineval.publish('/motion_execute', MotionPreviewMsg)
                    }
                }
            

            }

        },
        // Subscriptions
        [
            {'topic': '/debugMessage', 'type': 'DebugMessage'},
            {'topic': '/arm_position', 'type': 'ArmPosition'},
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

kineval.publish_joint_angles = function publish_joint_angles() {
    var ArmPositionMsg = {
        'type': 'ArmPosition',
        'joint_a': robot.joints['joint_a'].angle,
        'joint_b': robot.joints['joint_b'].angle,
        'joint_c': robot.joints['joint_c'].angle,
        'joint_d': robot.joints['joint_d'].angle,
        'joint_e': robot.joints['joint_e'].angle
    }
    kineval.publish('/ik_ra_control', ArmPositionMsg)
}

kineval.publish_target_angles = function publish_target_angles(goal) {
    var TargetAngleMsg = {
        'type': 'TargetAngles',
        'a': goal[0],
        'b': goal[1],
        'c': goal[2],
        'd': goal[3],
        'e': goal[4],
        'f': goal[5]
    }
    kineval.publish('/target_angles', TargetAngleMsg)
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

        // toggled robot link display
        if (kineval.params.display_links) {
            // var tempmat = matrix_mathjs_to_threejs(robot.links[x].xform);
            const geom = robot.links[x].geom
            const xform = robot.links[x].xform
            simpleApplyMatrix(robot.links[x].geom, robot.links[x].xform);
            robot.links[x].geom.visible = true;
        }
        else
            robot.links[x].geom.visible = false;

        // toggled robot link axes display
        if (kineval.params.display_links_axes) {
            robot.links[x].axis_geom_x.visible = true;
            robot.links[x].axis_geom_y.visible = true;
            robot.links[x].axis_geom_z.visible = true;
        }
        else {
            robot.links[x].axis_geom_x.visible = false;
            robot.links[x].axis_geom_y.visible = false;
            robot.links[x].axis_geom_z.visible = false;
        }

        // toggled robot link collision bounding box display
        if (kineval.params.display_collision_bboxes)
            robot.links[x].bbox_mesh.visible = true;
        else
            robot.links[x].bbox_mesh.visible = false;
    }

    // display bounding box for robot link in collision
    if (robot.collision)
        robot.links[robot.collision].bbox_mesh.visible = true;

    // toggled display of robot base axes
    if (kineval.params.display_base_axes) {
            robot.links[robot.base].axis_geom_x.visible = true;
            robot.links[robot.base].axis_geom_y.visible = true;
            robot.links[robot.base].axis_geom_z.visible = true;
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

        // toggled robot joint axes display
        if (kineval.params.display_joints_axes) {
            robot.joints[x].axis_geom_x.visible = true;
            robot.joints[x].axis_geom_y.visible = true;
            robot.joints[x].axis_geom_z.visible = true;
        }
        else {
            robot.joints[x].axis_geom_x.visible = false;
            robot.joints[x].axis_geom_y.visible = false;
            robot.joints[x].axis_geom_z.visible = false;
        }

    }

    // toggled display of joint with active control focus
    if (kineval.params.display_joints_active) {
        x = kineval.params.active_joint;
        // var tempmat = robot.joints[x].xform;
        simpleApplyMatrix(robot.joints[x].geom, robot.joints[x].xform);
        robot.joints[x].geom.visible = true;
        if (kineval.params.display_joints_active_axes) {
            robot.joints[x].axis_geom_x.visible = true;
            robot.joints[x].axis_geom_y.visible = true;
            robot.joints[x].axis_geom_z.visible = true;
        }
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
    var three_d_rot = new THREE.Matrix4().makeRotationX(kineval.params.ik_target.orientation[0])
    three_d_rot.multiply(new THREE.Matrix4().makeRotationY(kineval.params.ik_target.orientation[1]))
    three_d_rot.multiply(new THREE.Matrix4().makeRotationZ(kineval.params.ik_target.orientation[2]))

    var trans = new THREE.Matrix4().makeTranslation(kineval.params.ik_target.position[0][0],
                                                kineval.params.ik_target.position[1][0],
                                                kineval.params.ik_target.position[2][0])

    var target_mat = new THREE.Matrix4().multiplyMatrices(trans, three_d_rot)
    simpleApplyMatrix(target_geom,target_mat); // moving AND orienting green cube
    simpleApplyMatrix(cart_controller_x1_pos,target_mat);
    simpleApplyMatrix(cart_controller_y1_pos,target_mat);
    simpleApplyMatrix(cart_controller_z1_pos,target_mat);

    textbar.innerHTML = kineval.params.ik_target.position[0][0].toString()
    textbar.innerHTML += " " + kineval.params.ik_target.position[1][0].toString()
    textbar.innerHTML += " " + kineval.params.ik_target.position[2][0].toString()
    textbar.innerHTML += " (" + kineval.params.ik_target.orientation[0].toString()
    textbar.innerHTML += ", " + kineval.params.ik_target.orientation[1].toString()
    textbar.innerHTML += ", " + kineval.params.ik_target.orientation[2].toString() + ")"
    
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

    // create events and handlers for interaction controls
    kineval.initKeyEvents();


    // create GUI display object and configure
    kineval.initGUIDisplay();

}

kineval.initParameters = function initParameters() {

    // create params object
    kineval.params = {};

    kineval.params.just_starting = true;  // set to true as default, set false once starting forward kinematics project
    kineval.params.arm_enabled = false;
    // sets request for single update or persistent update of robot pose based on IK, setpoint controller, etc.
    kineval.params.update_pd = false;
    kineval.params.persist_pd = false;
    kineval.params.update_pd_clock = false;
    kineval.params.update_pd_dance = false;
    kineval.params.update_ik = false;
    kineval.params.persist_ik = false;


    // initialize the active joint for user control
    kineval.params.active_link = robot.base;
    //kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];

    if (typeof robot.links[kineval.params.active_link].children === 'undefined')
        kineval.params.active_joint = Object.keys(robot.joints)[0]
    else
        kineval.params.active_joint = robot.links[kineval.params.active_link].children[0];

    // initialize pose setpoints and target setpoint
    kineval.setpoints = [];
    kineval.params.setpoint_target = {};
    for (var i=0;i<10;i++) {  // 10 is the number of slots for pose setpoints
        kineval.setpoints[i] = {};
        for (x in robot.joints) {
            kineval.params.setpoint_target[x] = 0;  // current setpoint target
            kineval.setpoints[i][x] = 0;  // slot i setpoint
        }
    }

    // initialize inverse kinematics target location
    // KE 3 : ik_target param is redundant as an argument into inverseKinematics
    kineval.params.ik_target = {};
    kineval.params.ik_target.position = [[0],[0.8],[1.0],[1]];
    kineval.params.ik_target.orientation = [Math.PI/6, Math.PI/4, 0];
    kineval.params.ik_orientation_included = false;
    kineval.params.ik_steplength = 0.1;
    kineval.params.ik_pseudoinverse = false;

    // initialize flags for executing planner
    kineval.params.generating_motion_plan = false; // monitor specifying state of motion plan generation
    kineval.params.update_motion_plan = false; // sets request to generate motion plan
    kineval.motion_plan = [];
    kineval.motion_plan_traversal_index = 0;
    kineval.params.update_motion_plan_traversal = false; // sets automatic traversal of previously generated motion plan
    kineval.params.persist_motion_plan_traversal = false; // sets automatic traversal of previously generated motion plan
    kineval.params.planner_state = "not invoked";

    // toggle display of robot links, joints, and axes
    kineval.params.display_links = true;
    kineval.params.display_links_axes = false;
    kineval.params.display_base_axes = false;
    kineval.params.display_joints = false;
    kineval.params.display_joints_axes = false;
    kineval.params.display_collision_bboxes = false;
    kineval.params.display_wireframe = false;
    kineval.params.display_joints_active = true;
    kineval.params.display_joints_active_axes = true;

    // apply environment floor with map texture-mapped onto ground plane
    kineval.params.map_filename = url_params.map_filename;
    if (typeof kineval.params.map_filename === 'undefined') kineval.params.display_map = false;
    else kineval.params.display_map = true;

    // simulation_mode is true for not using the real arm
    // simulation_mode is false for using the real arm
    kineval.params.simulation_mode = true;

    kineval.params.lock_joint_e = true;
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
    camera_controls = new THREE.OrbitControls( camera );
    camera_controls.addEventListener( 'change', renderer );

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
    temp_geom = new THREE.CubeGeometry(0.1, 0.1, 0.1);
    temp_material = new THREE.MeshBasicMaterial( {color: 0x00ff00} ) // green cube
    target_geom = new THREE.Mesh(temp_geom, temp_material); // comment this for coolness
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

    var gui = new dat.GUI();

    dummy_display = {};
    dummy_display['kineval'] = function() {kineval.displayHelp};
    gui.add(dummy_display, 'kineval');

    gui.add(kineval.params, 'arm_enabled').onChange(function () {
        var TalonConfigMsg =  {
            'type': 'TalonConfig',
            'enable_arm': kineval.params.arm_enabled,
            'enable_sa': false,
        }
        kineval.publish('/talon_config', TalonConfigMsg)

        for (var i = 1; i <= 6; i++) {
            var OpenLoopMsg = {
                'type': 'OpenLoopRAMotor',
                'joint_id': i,
                'speed': 0
            }
            kineval.publish('/arm_motors', OpenLoopMsg)
        }
    });

    gui.add(kineval.params, 'simulation_mode').onChange(function () {
        var SimulationModeMsg = {
            'type': 'SimulationMode',
            'sim_mode': kineval.params.simulation_mode
        }
        kineval.publish('/simulation_mode', SimulationModeMsg)
    });

    gui.add(kineval.params, 'lock_joint_e').onChange(function () {
        var LockJointEMsg = {
            'type': 'LockJointE',
            'locked': kineval.params.lock_joint_e
        }
        kineval.publish('/lock_joint_e', LockJointEMsg)
    });

    var dummy_object = {};
    dummy_object.send_target_orientation = function() {
        
        var three_d_rot = new THREE.Matrix4().makeRotationX(kineval.params.ik_target.orientation[0])
        three_d_rot.multiply(new THREE.Matrix4().makeRotationY(kineval.params.ik_target.orientation[1]))
        three_d_rot.multiply(new THREE.Matrix4().makeRotationZ(kineval.params.ik_target.orientation[2]))

        var alph = Math.atan2(three_d_rot.elements[2], -(three_d_rot.elements[6]));
        var bet = Math.acos(three_d_rot.elements[10]);
        var gam = Math.atan2(three_d_rot.elements[8], three_d_rot.elements[9]);

        var TargetOrientationMsg =  {
            'type': 'TargetOrientation',
            'x': kineval.params.ik_target.position[0][0],
            'y': -1 * kineval.params.ik_target.position[2][0],
            'z': kineval.params.ik_target.position[1][0],
            // 'alpha': alph,
            // 'beta': bet,
            // 'gamma': gam,
            'alpha': 0,
            'beta': 0,
            'gamma': 0,
        }

        kineval.publish('/target_orientation', TargetOrientationMsg)
        kineval.params.update_motion_plan = true; 
        console.log("sent point")
    }

    dummy_object.target_angle_neutral = function() {
        var TargetOrientationMsg =  {
            'type': 'TargetOrientation',
            'x': 0.0,
            'y': 0.5,
            'z': 1.0,
            'alpha': 0.1,
            'beta': 0.0,
            'gamma': 0.0,
        }
        kineval.publish('/target_orientation', TargetOrientationMsg)
    }

    dummy_object.target_angle_down = function() {
        var TargetOrientationMsg =  {
            'type': 'TargetOrientation',
            'x': 0.0,
            'y': 0.3,
            'z': 1.5,
            'alpha': 1.3,
            'beta': 0.0,
            'gamma': 0.0,
        }
        kineval.publish('/target_orientation', TargetOrientationMsg)
    }

    dummy_object.preview_plan = function() {
        var MotionPreviewMsg = {
            'type': 'MotionExecute',
            'preview': true,
        }
        console.log('Previewing plan')
        // textbar.innerHTML = "execute preview";
        kineval.publish('/motion_execute', MotionPreviewMsg)
    }

    dummy_object.execute_plan = function() {
        var MotionExecuteMsg = { 
            'type': 'MotionExecute',
            'preview': false,
        }
        // textbar.innerHTML = "execute motor controls";
        kineval.publish('/motion_execute', MotionExecuteMsg)
    }

    // 1. send point
    // 2. preview 
    // 3. execute 
    gui.add(dummy_object, 'send_target_orientation');
    gui.add(dummy_object, 'target_angle_neutral');
    gui.add(dummy_object, 'target_angle_down');
    gui.add(dummy_object, 'preview_plan');
    gui.add(dummy_object, 'execute_plan')


}

kineval.initRobotLinksGeoms = function initRobotLinksGeoms() {

    // KE T: initialize this variable properly
    robot.collision = false;

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

        // For collision detection,
        // set the bounding box of robot link in local link coordinates
        robot.links[x].bbox = new THREE.Box3;
        //(THREE r62) robot.links[x].bbox = robot.links[x].bbox.setFromPoints(robot.links[x].geom.geometry.vertices);
        // setFromObject returns world space bbox
        robot.links[x].bbox = robot.links[x].bbox.setFromObject(robot.links[x].geom);
        // setFromPoints returns local space bbox, but no child traversal
        //robot.links[x].bbox = robot.links[x].bbox.setFromPoints(robot.links[x].geom.geometry.vertices);

        /* (73) (does not consider origin offset)
        bbox_geom = new THREE.BoxGeometry(
            robot.links[x].bbox.max.x-robot.links[x].bbox.min.x,
            robot.links[x].bbox.max.y-robot.links[x].bbox.min.y,
            robot.links[x].bbox.max.z-robot.links[x].bbox.min.z
        );
        */

        // (92) need to add bbox geometry directly
        var bbox_geom = new THREE.Geometry();
        bbox_geom.vertices = []; // for some reason, the allocation above populates the vertices array of the geometry with the dimensions of a bbox
        bbox_geom.vertices.push(
            new THREE.Vector3(robot.links[x].bbox.min.x,robot.links[x].bbox.min.y,robot.links[x].bbox.min.z),
            new THREE.Vector3(robot.links[x].bbox.min.x,robot.links[x].bbox.min.y,robot.links[x].bbox.max.z),
            new THREE.Vector3(robot.links[x].bbox.min.x,robot.links[x].bbox.max.y,robot.links[x].bbox.min.z),
            new THREE.Vector3(robot.links[x].bbox.min.x,robot.links[x].bbox.max.y,robot.links[x].bbox.max.z),
            new THREE.Vector3(robot.links[x].bbox.max.x,robot.links[x].bbox.min.y,robot.links[x].bbox.min.z),
            new THREE.Vector3(robot.links[x].bbox.max.x,robot.links[x].bbox.min.y,robot.links[x].bbox.max.z),
            new THREE.Vector3(robot.links[x].bbox.max.x,robot.links[x].bbox.max.y,robot.links[x].bbox.min.z),
            new THREE.Vector3(robot.links[x].bbox.max.x,robot.links[x].bbox.max.y,robot.links[x].bbox.max.z)
        );

        bbox_geom.faces.push(
            new THREE.Face3(0,1,2),
            new THREE.Face3(1,3,2),
            new THREE.Face3(4,5,6),
            new THREE.Face3(5,7,6),
            new THREE.Face3(1,5,7),
            new THREE.Face3(1,7,6),
            new THREE.Face3(2,3,7),
            new THREE.Face3(2,7,6),
            new THREE.Face3(0,4,6),
            new THREE.Face3(0,6,2),
            new THREE.Face3(0,1,4),
            new THREE.Face3(1,3,4)
        );


        bbox_material = new THREE.MeshBasicMaterial( { color: 0xFF0000, wireframe:true, visible:true } );

        // KE 2 : move bbox_mesh to proper place within link object
        robot.links[x].bbox_mesh = new THREE.Mesh(bbox_geom,bbox_material);
        robot.links[x].geom.add(robot.links[x].bbox_mesh);

        // xyz axis indicators
        axis_geom_x = new THREE.Geometry();
        axis_geom_x.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(1,0,0)
        );
        robot.links[x].axis_geom_x = new THREE.Line(axis_geom_x,
            new THREE.LineBasicMaterial({color: 0xFF0000}));
        robot.links[x].geom.add(robot.links[x].axis_geom_x);

        axis_geom_y = new THREE.Geometry();
        axis_geom_y.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(0,1,0)
        );
        robot.links[x].axis_geom_y = new THREE.Line(axis_geom_y,
            new THREE.LineBasicMaterial({color: 0x00FF00}));
        robot.links[x].geom.add(robot.links[x].axis_geom_y);

        axis_geom_z = new THREE.Geometry();
        axis_geom_z.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(0,0,1)
        );
        robot.links[x].axis_geom_z = new THREE.Line(axis_geom_z,
            new THREE.LineBasicMaterial({color: 0x0000FF}));
        robot.links[x].geom.add(robot.links[x].axis_geom_z);

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

        // KE 3 : vary axis size
        axis_geom_x = new THREE.Geometry();
        axis_geom_x.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(1,0,0)
        );
        robot.joints[x].axis_geom_x = new THREE.Line(axis_geom_x,
            new THREE.LineBasicMaterial({color: 0xFF0000}));
        robot.joints[x].geom.add(robot.joints[x].axis_geom_x);

        axis_geom_y = new THREE.Geometry();
        axis_geom_y.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(0,1,0)
        );
        robot.joints[x].axis_geom_y = new THREE.Line(axis_geom_y,
            new THREE.LineBasicMaterial({color: 0x00FF00}));
        robot.joints[x].geom.add(robot.joints[x].axis_geom_y);

        axis_geom_z = new THREE.Geometry();
        axis_geom_z.vertices.push(
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(0,0,1)
        );
        robot.joints[x].axis_geom_z = new THREE.Line(axis_geom_z,
            new THREE.LineBasicMaterial({color: 0x0000FF}));
        robot.joints[x].geom.add(robot.joints[x].axis_geom_z);


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
/////     CONTROLLER INTERFACE FUNCTIONS
//////////////////////////////////////////////////

kineval.setPoseSetpoint = function set_pose_setpoint (pose_id) {
    if (pose_id < 1)
        textbar.innerHTML = "setpoint is preset zero pose";
    else
        textbar.innerHTML = "setpoint is user defined pose "+pose_id;
    kineval.params.setpoint_id = pose_id;
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = kineval.setpoints[pose_id][x];
    }
}

kineval.assignPoseSetpoint = function assign_pose_setpoint (pose_id) {
    if ((pose_id < 1)||(pose_id>9))
        console.warn("kineval: setpoint id must be between 1 and 9 inclusive");
    else
        textbar.innerHTML = "assigning current pose to setpoint "+pose_id;
    for (x in robot.joints) {
        kineval.setpoints[pose_id][x] = robot.joints[x].angle;
    }
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
