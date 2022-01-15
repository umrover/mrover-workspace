#include <vector>
#include <iostream>


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
    std::vector<std::vector<float>> discrete_truths; //%-detected for each truth obs

  private: //(non-parametric) instance variables
    std::vector<float> avg_truth_detected; // avg truth %detected

  private: //utility FUNCTIONS
      auto analyze_test() -> void;//Run statistical analysis


  public:
      TestStats();
      TestStats(std::vector<float> iot_, std::vector<float> fot_, std::vector<float> times_, std::vector<int> nt_, std::vector<int> nd, std::vector<std::vector<float>> dscrt);

      //auto getTrue() const -> const std::vector<EuclideanClusterExtractor::ObsReturn>&;
      //auto getDet()  const -> const std::vector<EuclideanClusterExtractor::ObsReturn>&;

      auto print()          -> void; //prints out all info
      auto print(bool fast) -> void; //option for fast print

};
