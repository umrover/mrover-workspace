<script>
  import { Line, mixins } from 'vue-chartjs'

  const time=10;
  const refresh=0.1;
  export default {
    extends: Line,

    mixins: [mixins.reactiveProp],

    props: {
      chartData: {
        type: Array,
        required: true
      }
    },

    methods: {
      renderPIDChart: function() {
        this.renderChart({
          datasets: this.chartData
        },
        {
          animation: {
            duration: 0, // general animation time
          },
          hover: {
            animationDuration: 0, // duration of animations when hovering an item
          },
          responsiveAnimationDuration: 0, // animation duration after a resize

          scales:{
            xAxes:[{
              type: 'linear',
              id: 'x-axis',
              display: true,
              ticks:{
                min: 0,
                max: time
              }
            }],

            yAxes:[{
              type: 'linear',
              id: 'y-axis',
              ticks:{
                min: -10240,
                max: 10240,
                stepSize: 640
              }
            }]
          }
        })
      }
    }
  }
</script>
