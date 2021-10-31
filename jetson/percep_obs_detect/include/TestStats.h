#include <vector>
#include <iostream>
class TestStats {
  private:
    vector<float> iot; //intersection of true over total true vol
    vector<float> fot; //false positive vol over total true vol
    vector<float> times;
    vector<int> num_true_obs;
    vector<int> num_det_obs;

  public:
      TestStats()
      {

      }

      TestStats(vector<float> iot_, vector<float> fot_, vector<float> times_, vector<int> nt, vector<int> nd) :
      iot(iot_), fot(fot_), times(times_), num_true_obs(nt_), num_det_obs(nd){}

      void print()
      {
        cout << "Evaluating GPU Cloud #[NUMBER]\n";
        cout << "GPU Obstacle Detection Runtime: [TIME]\n";
        cout << "Number of Detected Obstacles: [NUMDET]\n";
        cout << "Number of True Obstacles: [NUMTRUE]\n";
        cout << "\t(What Should be detected)\n";
        cout << "Percent Truth Detected: [TRUE]\n";
        cout << "\t(Total intersection divided by total true area)\n\tShould be clsoe to 1\n"
        cout << "False Positive over True Volume: [FALSE]\n";
        cout << "\t(Total detected volume not contained in set of true obstacles)\n\t(Divided by total true volume)\n";

        for(size_t i = 0; i < iot.size(); i++)
        {
          cout << "\n–––––––––––––––––––––––\n–––––––––––––––––––––––\n\n"
          cout << "Evaluating GPU Cloud #" << i << "\n";
          cout << "GPU Obstacle Detection Runtime: " << times[i]; << "\n";
          cout << "Number of Detected Obstacles: " << num_det_obs[i] << "\n";
          cout << "Number of True Obstacles: " << num_true_obs[i] << "\n";
          cout << "Percent Truth Detected: " << iot[i] << "\n";
          cout << "False Positive over True Volume: " << fot[i] << "\n";

        }
      }

};
