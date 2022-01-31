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
                    <button class="zero-button" v-on:click="zero_position_callback()">
                        Set Zero Position
                    </button>
                </div>
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
                <button class="preset-button" v-on:click="preset_position_callback('stowed')">
                    Stowed
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

        updateJointsLocked: function(joint, locked) {
            this.locked_joints[joint] = locked;

            var lockedJointsMsg = this.locked_joints;
            lockedJointsMsg['type'] = 'LockJoints';

            this.$parent.publish('/locked_joints', lockedJointsMsg);
        },

        zeroPositionCallback: function() {
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

.preset-button {
    display: flex;
    justify-content: space-between;
}


</style>
