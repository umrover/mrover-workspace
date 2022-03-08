<template>
    <div>
        <h3>IK Controls</h3>
        <div class="ik-toggles">
            <div class="config-toggles">
                <div class="safe-toggles">
                    <Checkbox ref="sim-mode" v-bind:name="'Sim Mode'" v-on:toggle="updateSimMode($event)"/>
                    <Checkbox ref="use-orientation" v-bind:name="'Use Orientation'" v-on:toggle="updateUseOrientation($event)"/>
                </div>
                <div class="zero-position-wrapper">
                    <button class="zero-button" v-on:click="zeroPositionCallback()">
                        Set Zero Position
                    </button>
                </div>
            </div>

            <h4>Increment Control</h4>
            <div class="increment-control">
                <button type="button" v-on:click="sendIncrement()">Send</button>
                <label style="float: left" for="'x-inc'">R/L :  </label>
                <input class="increment-box" id='x-inc' type="text" v-model='x_inc' />
                <label style="float: left" for="'y-inc'">F/B :  </label>
                <input class="increment-box" id='y-inc' type="text" v-model='y_inc' />
                <label style="float: left" for="'z-inc'">U/D :  </label>
                <input class="increment-box" id='z-inc' type="text" v-model='z_inc' />
                <label style="float: left" for="'alpha-inc'">alpha :  </label>
                <input class="increment-box" id='alpha-inc' type="text" v-model='alpha_inc' />
                <label style="float: left" for="'beta-inc'">beta :  </label>
                <input class="increment-box" id='beta-inc' type="text" v-model='beta_inc' />
                <label style="float: left" for="'gamma-inc'">gamma :  </label>
                <input class="increment-box" id='gamma-inc' type="text" v-model='gamma_inc' />
            </div>

            <h4>Joint Locks</h4>
            <div class="joint-locks">
                <Checkbox ref="joint-a-lock" v-bind:name="'Lock A'" v-on:toggle="updateJointsLocked('joint_a', $event)"/>
                <Checkbox ref="joint-b-lock" v-bind:name="'Lock B'" v-on:toggle="updateJointsLocked('joint_b', $event)"/>
                <Checkbox ref="joint-c-lock" v-bind:name="'Lock C'" v-on:toggle="updateJointsLocked('joint_c', $event)"/>
                <Checkbox ref="joint-d-lock" v-bind:name="'Lock D'" v-on:toggle="updateJointsLocked('joint_d', $event)"/>
                <Checkbox ref="joint-e-lock" v-bind:name="'Lock E'" v-on:toggle="updateJointsLocked('joint_e', $event)"/>
                <Checkbox ref="joint-f-lock" v-bind:name="'Lock F'" v-on:toggle="updateJointsLocked('joint_f', $event)"/>
            </div>

            <h4>Preset Position</h4>
            <div class="presets">
                <button class="preset-button" v-on:click="presetPositionCallback('stowed')">
                    Stowed
                </button>
                <button class="preset-button" v-on:click="presetPositionCallback('weight_in')">
                    Weight In
                </button>
            </div>
        </div>
    </div>
</template>


<script>
import Checkbox from './Checkbox.vue'

export default {

    data () {
        return {
            sim_mode: true,
            use_orientation: false,

            x_inc: 0.0,
            y_inc: 0.0,
            z_inc: 0.0,
            alpha_inc: 0.0,
            beta_inc: 0.0,
            gamma_inc: 0.0,

            locked_joints: {
                'joint_a': false,
                'joint_b': false,
                'joint_c': false,
                'joint_d': false,
                'joint_e': false,
                'joint_f': false
            }
        }
    },

    methods: {
        updateSimMode: function(checked) {
            const simModeMsg = {
                'type': 'SimulationMode',
                'sim_mode': checked
            }
            this.$parent.publish('/simulation_mode', simModeMsg);
        },

        updateUseOrientation: function(checked) {
            const useOrientationMsg = {
                'type': 'UseOrientation',
                'use_orientation': checked
            }
            this.$parent.publish('/use_orientation', useOrientationMsg);
        },

        sendIncrement: function() {
            const msg = {
                'type': 'ArmAdjustments',
                'x': parseFloat(this.x_inc),
                'y': parseFloat(this.y_inc),
                'z': parseFloat(this.z_inc),
                'alpha': parseFloat(this.alpha_inc),
                'beta': parseFloat(this.beta_inc),
                'gamma': parseFloat(this.gamma_inc)
            }
            this.$parent.publish('/arm_adjustments', msg);
        },

        updateJointsLocked: function(joint, locked) {
            this.locked_joints[joint] = locked;

            var lockedJointsMsg = this.locked_joints;
            lockedJointsMsg['type'] = 'LockJoints';

            this.$parent.publish('/locked_joints', lockedJointsMsg);
        },

        zeroPositionCallback: function() {
	    console.log("publishing zero position")
            this.$parent.publish('/zero_position', { 'type': 'ZeroPosition' });
        },

        presetPositionCallback: function(position) {
            this.$parent.publish('/arm_preset', { 'type': 'ArmPreset', 'preset': position });
        }
    },

    components: {
        Checkbox
    }
}

</script>

<style scoped>

.config-toggles {
    display: flex;
    justify-content: space-between;
}

.safe-toggles {
    display: flex;
    justify-content: begin;
}

.zero-position-wrapper {
    display: flex;
    justify-content: end;
}

.joint-locks {
    display: flex;
    justify-content: space-between;
}

.increment-control {
    display: flex;
    justify-content: space-between;
}

.increment-box {
    width: 20px;
}

.presets {
    display: flex;
    justify-content: left;
}

.preset-button {
    margin: 0px 10px;
}


</style>
