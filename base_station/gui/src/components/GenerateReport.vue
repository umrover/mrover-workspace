<template>
    <div>
        <button v-on:click="download_csv_file()">Generate Report</button>
    </div>
</template>

<script>
export default {
    data () {
        return {
            temp0:0,
            temp1:0,
            temp2:0,
            csvFileData:[]
        }
    },
  props: {
    spectral_data: {
      type: Object,
      required: true
    },
  },
  created:
    function () {
      this.$parent.subscribe('/thermistor_data', (msg) => {
        this.temp0 = msg.temp0
        this.temp1 = msg.temp1
        this.temp2 = msg.temp2
      })
    },
    methods:{    
    download_csv_file: function() {  

        this.csvFileData = [  
        ['Alan Walker', 'Singer'],  
        ['Cristiano Ronaldo', 'Footballer'],  
        ['Saina Nehwal', 'Badminton Player'],  
        ['Arijit Singh', 'Singer'],  
        ['Terence Lewis', 'Dancer']  
        ];  
        //define the heading for each row of the data  
        var csv = 'Name,Profession\n';  
        
        //merge the data with CSV  
        this.csvFileData.forEach(function(row) {  
                csv += row.join(',');  
                csv += "\n";  
        });  
          
        
        var hiddenElement = document.createElement('a');  
        hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);  
        hiddenElement.target = '_blank';  
        
        //provide the name for the CSV file to be downloaded  
        hiddenElement.download = 'Famous Personalities.csv';  
        hiddenElement.click();  
    }  
    }
}
</script>

<style scoped>

</style>