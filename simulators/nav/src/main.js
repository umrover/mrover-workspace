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
        if (topic === '/joystick') {
            app.apply_joystick(message);
        } else if (topic === '/nav_status') {
            app.set(message);
        }
    },
    // Subscriptions
    [{ 
        'topic': '/joystick', 
        'type': 'Joystick'
    },
    {
        'topic': '/nav_status',
        'type': 'NavStatus'
    }]
)

app.on("odom", (odom) => {
    if (bridge.online) {
        odom.type = 'Odometry';
        bridge.publish('/odom', odom);
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

app.on("tennis_ball", (tennis_ball) => {
    if (bridge.online) {
        tennis_ball.type = 'TennisBall';
        bridge.publish("/tennis_ball", tennis_ball);
    }
})

app.start_odom_events();
