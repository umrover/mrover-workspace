<template>
<div class="status wrap" v-bind:style="{backgroundColor: current_background}">
    <div>
        <h3> Strip Test Controls </h3>
    </div>
    <div class="controls">
        <label for="strip">Perform strip test on strip:</label>
        <select v-model="current_strip" name="strip" id="strip">
            <option value=0>0</option>
            <option value=1>1</option>
            <option value=2>2</option>
        </select>
        <div class="commands" v-if="!servos[current_strip].has_dipped">
            <button v-on:click="stripTest(current_strip)"> start strip {{ current_strip }} test </button>
        </div>
        <button v-on:click="back_to_start()">Back to start</button>
        <div class="input">
            Angle: <input type='number' v-model.number="custom_angle">
            <button v-on:click="userInput()">Move to angle</button>
        </div>
    </div>
</div>
</template>

<script>
const start_position = [80, 100, 100]
const dip_position = [10, 30, 30]

const undipped = '#A54657'
const dipping = '#FABC2A'
const dipped = '#AEF78E'
const unknown = '#EEEEEE'

export default {
    data () {
        return {
            current_strip: 0,
            current_background: undipped,

            custom_angle: 0,

            servos: [
                {
                    angle: start_position[0],
                    has_dipped: false,
                    color: undipped
                },
                {
                    angle: start_position[1],
                    has_dipped: false,
                    color: undipped
                },
                {
                    angle: start_position[2],
                    has_dipped: false,
                    color: undipped
                }
            ]
        }
    },

    methods: {
        sendCommand: function() {
            this.$parent.publish("/servo_cmd", {
                'type': 'ServoCmd',
                'angle0': this.servos[0].angle,
                'angle1': this.servos[1].angle,
                'angle2': this.servos[2].angle
            })
        },

        stripTest: function(id) {
            if (this.servos[id].has_dipped) {
                return
            }
            this.servos[id].has_dipped = true
            this.servos[id].angle = dip_position[id]
            this.servos[id].color = dipping
            this.current_background = dipping

            this.sendCommand()

            setTimeout(() => {
                if (this.servos[id].angle === dip_position[id]) {
                    this.servos[id].angle = start_position[id]
                    this.servos[id].color = dipped
                    this.current_background = dipped

                    this.sendCommand()
                }
            }, 10000)
        },

        back_to_start: function() {
            this.updateCurrentStrip(start_position[this.current_strip], false, undipped)
        },

        userInput: function() {
            this.updateCurrentStrip(this.custom_angle, true, unknown)
        },

        updateCurrentStrip: function(angle, has_dipped, color) {
            this.servos[this.current_strip].angle = angle
            this.servos[this.current_strip].has_dipped = has_dipped
            this.servos[this.current_strip].color = color
            this.current_background = color

            this.sendCommand()
        }
    },

    watch: {
        current_strip: function(val) {
            this.current_background = this.servos[val].color
        }
    }
}
</script>

<style scoped>
    .wrap {
        display: inline-block;
    }

    .controls {
        display: inline-block;
        vertical-align: top;
        align-content: left;
        padding: 4%;
    }

    .commands{
        text-align: left;
        display: inline-block;
        padding: 4%;
    }

    .input {
        padding-bottom: 4%;
    }
</style>
