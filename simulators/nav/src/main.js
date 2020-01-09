import App from './components/App.html';
import LCMBridge from 'bridge';

const app = new App({
    target: document.querySelector('main')
});

const bridge = new LCMBridge("ws://localhost:8001",
    // Is websocket online?
    (online) => {
        if (!online) {
            app.set({
                'lcm_connected': false
            });
        } else {
            bridge.setHomePage();
        }
    },
    // Is LCM online?
    (online) => {
        app.set({
            'lcm_connected': online
        });
    },
    // Message handler
    ({topic, message}) => {
        if (topic === '/autonomous') {
            app.apply_joystick(message);
        } else if (topic === '/nav_status') {
            app.set(message);
        }
    },
    // Subscriptions
    [{
        'topic': '/autonomous',
        'type': 'Joystick'
    },
    {
        'topic': '/nav_status',
        'type': 'NavStatus'
    }]
)

app.on("imu", (imu) => {
    if (bridge.online) {
        imu.type = 'IMU';
        bridge.publish('/imu', imu)
    }
})

app.on("gps", (gps) => {
    if (bridge.online) {
        gps.type = 'GPS';
        bridge.publish('/gps', gps);
    }
})

app.on("sensorPackage", (sensorPackage) => {
    if (bridge.online) {
        sensorPackage.type = 'SensorPackage';
        bridge.publish('/sensor_package', sensorPackage);
    }
})

app.on("course", (course) => {
    if (bridge.online) {
        course.type = 'Course';
        bridge.publish('/course', course);
    }
})

app.on("auton", (auton) => {
    if (bridge.online) {
        auton.type = 'AutonState';
        bridge.publish('/auton', auton);
    }
})

app.on("target_list", (target_list) => {
    if (bridge.online) {
        target_list.type = 'TargetList';
        bridge.publish("/target_list", target_list);
    }
})

app.on("obstacle", (obstacle) => {
    if (bridge.online) {
        obstacle.type = 'Obstacle';
        bridge.publish("/obstacle", obstacle);
    }
})

app.start_odom_events();
