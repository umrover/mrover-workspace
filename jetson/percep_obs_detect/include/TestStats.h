#include <vector>
#include <iostream>
class TestStats {
  private:
    std::vector<float> iot; //intersection of true over total true vol
    std::vector<float> fot; //false positive vol over total true vol
    std::vector<float> times; //gpu obs detect runtimes
    std::vector<int> num_true_obs; //number of true obstacles
    std::vector<int> num_det_obs; //number of test obstacles
    std::vector<vector<float>> discrete_truths; //%-detected for each truth obs

  private:
    float avg_pct_truth()
    {
      for(size_t i = 0; i < discrete_truths.size(); i++)
      {
        for(size_t j = 0;)
      }
    }

  public:
      TestStats()
      {

      }

      TestStats(std::vector<float> iot_, std::vector<float> fot_, std::vector<float> times_, std::vector<int> nt, std::vector<int> nd, std::vector<vector<float>> dscrt) :
      iot(iot_), fot(fot_), times(times_), num_true_obs(nt_), num_det_obs(nd), discrete_truths(dscrt){}

      void print()
      {
        std::cout.setprecision(4);
        std::cout << "Evaluating GPU Cloud #[NUMBER]\n";
        std::cout << "GPU Obstacle Detection Runtime: [TIME]\n";
        std::cout << "Number of Detected Obstacles: [NUMDET]\n";
        std::cout << "Number of True Obstacles: [NUMTRUE]\n";
        std::cout << "\t(What Should be detected)\n";
        std::cout << "Percent Truth Detected: [TRUE]\n";
        std::cout << "\t(Total intersection divided by total true area)\n\tShould be clsoe to 1\n"
        std::cout << "False Positive over True Volume: [FALSE]\n";
        std::cout << "\t(Total detected volume not contained in set of true obstacles)\n\t(Divided by total true volume)\n";

        for(size_t i = 0; i < iot.size(); i++)
        {
          std::cout << "\n–––––––––––––––––––––––\n–––––––––––––––––––––––\n\n"
          std::cout << "Evaluating GPU Cloud #" << i << "\n";
          std::cout << "GPU Obstacle Detection Runtime: " << times[i]; << "\n";
          std::cout << "Number of Detected Obstacles: " << num_det_obs[i] << "\n";
          std::cout << "Number of True Obstacles: " << num_true_obs[i] << "\n";
          std::cout << "Percent Truth Detected: " << iot[i] << "\n";
          std::cout << "False Positive over True Volume: " << fot[i] << "\n";
        }

      }

};
