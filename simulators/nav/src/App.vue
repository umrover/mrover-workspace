<!-- This file contains the App component which is the root of the Vue program.
     It contains the router which directs users to the correct page within the
     application. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div id="app">
    <router-view />
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({})
export default class App extends Vue {}
</script>


<!--------------------------------------------- Style --------------------------------------------->
<style>
body {
  margin: 0;
}
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  height: 99vh;
  width: 99.5vw;
  margin: 0.5vh 0.25vw;
}

.box {
  flex-grow: 1;
}

.box p {
  margin: auto;
}

.container {
  border: solid 2px black;
  border-radius: 5px;
  background-color: lightgray;
  padding: 5px;
}

.disabled,
:disabled {
  border-color: gray;
  color: gray;
  cursor: not-allowed;
  transition: 0.5s;
}

.draw-options-container p {
  display: inline-block;
}

fieldset {
  border-radius: 5px;
  border: solid 1px black;
  padding: 2px 5px;
}

input {
  border-radius: 4px;
  border: 2px solid #444;
  height: 20px;
  background-color: inherit;
  margin-top: 2px;
  text-align: center;
  font-size: 1em;
  margin: 2px;
  outline: none;
  line-height: 100%;
}

p.disabled {
  color: gray;
}

.off {
  background-color: rgb(255, 90, 90);
}

.off:hover {
  background-color: rgb(255, 120, 120);
}

.off p {
  color: white;
}

.off,
.off p,
.on,
.on p {
  transition: 0.5s;
}

.on {
  background-color: rgb(160, 255, 160);
}

.on:hover {
  background-color: rgb(190, 255, 190);
}
</style>
