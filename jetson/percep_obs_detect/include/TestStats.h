#include <vector>
#include <iostream>
#include <numeric>
#include <iomanip>

/* Brief:
** Each call of obs-detector::test() returns a TestStats obj;
** it's basically a vector of point clouds, which in turn is a vec of obs
*/
class TestStats {
  private: //paramatric instantiation
    std::vector<float> iot; //intersection of true over total true vol
    std::vector<float> fot; //false positive vol over total true vol
    std::vector<float> times; //gpu obs detect runtimes
    std::vector<int> num_true_obs; //number of true obstacles
    std::vector<int> num_det_obs; //number of test obstacles
    std::vector<vector<float>> discrete_truths; //%-detected for each truth obs

  private: //(non-parametric) instance variables
    std::vector<float> avg_truth_detected; // avg truth %detected


  public:
      TestStats()
      {

      }

      TestStats(std::vector<float> iot_, std::vector<float> fot_, std::vector<float> times_, std::vector<int> nt_, std::vector<int> nd, std::vector<vector<float>> dscrt) :
      iot(iot_), fot(fot_), times(times_), num_true_obs(nt_), num_det_obs(nd), discrete_truths(dscrt){}


      void print() //prints out all info
      {
        /* Output Log Format */
        //std::cout.setprecision(4);
        std::cout << "Evaluating GPU Cloud #[NUMBER]\n";
        std::cout << "GPU Obstacle Detection Runtime: [TIME]\n";
        std::cout << "Number of Detected Obstacles: [NUMDET]\n";
        std::cout << "Number of True Obstacles: [NUMTRUE]\n";
        std::cout << "\t(What Should be detected)\n";
        std::cout << "Percent Truth Detected: [TRUE]\n";
        std::cout << "\t(Total intersection divided by total true area)\n\tShould be clsoe to 1\n";
        std::cout << "False Positive over True Volume: [FALSE]\n";
        std::cout << "\t(Total detected volume not contained in set of true obstacles)\n\t(Divided by total true volume)\n";
        std::cout << "Mean % Truth: [TRUEPCT]\n"
        std::cout << "Average volume of a truth detected\n"


        for(size_t i = 0; i < iot.size(); i++) // Loop through each cloud
        {
          std::cout << "\n–––––––––––––––––––––––\n–––––––––––––––––––––––\n\n";
          std::cout << "Evaluating GPU Cloud #" << i << "\n";
          std::cout << "GPU Obstacle Detection Runtime: " << times[i] << "\n";
          std::cout << "Number of Detected Obstacles: " << num_det_obs[i] << "\n";
          std::cout << "Number of True Obstacles: " << num_true_obs[i] << "\n";
          std::cout << "Percent Truth Detected: " << iot[i] << "\n";
          std::cout << "False Positive over True Volume: " << fot[i] << "\n";
          std::cout << "Mean % Truth: " <<
            std::accumulate(discrete_truths[i].begin(),
                            discrete_truths[i].end(),
                            static_cast<float>(0),
                            [&](const float& a, const float& b){
                              return a + b / discrete_truths[i].size();
                            }) << "\n";
        }

      }//End print()

};
