<template>
   <div class = "wrapper">
       <div v-bind:style="{color: activeColor}">
        <span>PDB Current Draw: {{currentDraw}} amps</span><br>
        <span>PDB Temperature: {{temperature}} C°</span>
       </div>
   </div> 
</template>

<script>
import SpectralData from './SpectralData.vue'
export default {
  components: { SpectralData },
    data(){
        return{
            currentDraw:0,
            temperature:0,
            activeColor:"red"
        }
    },
    created: function(){
        setTimeout(() => {  this.currentDraw = 45;
                            this.temperature = 21;
                            this.activeColor= "green"; }, 10000);
    }

}
</script>

<style scoped>
.wrapper{
    padding: 0px;
    padding-left: 0px;
    padding-right: 0px;
    border: none;
    margin-top: 0.5rem;
}
</style>