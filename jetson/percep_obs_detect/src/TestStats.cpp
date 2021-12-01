#pragma once

#include "TestStats.h"
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <fstream>

TestStats::TestStats()
{

}


TestStats::TestStats(std::vector<float> iot_, std::vector<float> fot_, std::vector<float> times_, std::vector<int> nt_, std::vector<int> nd, std::vector<std::vector<float>> dscrt) :
iot(iot_), fot(fot_), times(times_), num_true_obs(nt_), num_det_obs(nd), discrete_truths(dscrt)
{
  this->print();
}


TestStats::~TestStats()
{

}

/*
void TestStats::analyze_test()
{
  std::for_each(discrete_truths.begin(),discrete_truths.end(),
    std::accumulate(discrete_truths[i]))
}
*/

void TestStats::print() //prints out all info
{
  std::ofstream of("jetson/percep_obs_detect/unit_test.txt");
    /* Output Log Format */
    //of.setprecision(4);
    of << "Evaluating GPU Cloud #[NUMBER]\n";
    of << "GPU Obstacle Detection Runtime: [TIME]\n";
    of << "Number of Detected Obstacles: [NUMDET]\n";
    of << "Number of True Obstacles: [NUMTRUE]\n";
    of << "\t(What Should be detected)\n";
    of << "Percent Truth Detected: [TRUE]\n";
    of << "\t(Total intersection divided by total true area)\n\tShould be close to 1\n";
    of << "False Positive over True Volume: [FALSE]\n";
    of << "\t(Total detected volume not contained in set of true obstacles)\n\t(Divided by total true volume)\n";
    of << "Mean % Truth: [TRUEPCT]\n";
    of << "Average volume of a truth detected\n";
    of << "Obstacle #[NUMBER] % detected: [PERCENT]\n";

    for (size_t i = 0; i < iot.size(); i++) // Loop through each cloud
    {
        of << "\n–––––––––––––––––––––––\n–––––––––––––––––––––––\n\n";
        of << "Evaluating GPU Cloud #" << i << "\n";
        of << "GPU Obstacle Detection Runtime: " << times[i] << "\n";
        of << "Number of Detected Obstacles: " << num_det_obs[i] << "\n";
        of << "Number of True Obstacles: " << num_true_obs[i] << "\n";
        of << "Percent Truth Detected: " << iot[i] * 100 << "\n";
        of << "False Positive over True Volume: " << fot[i] << "\n";
        of << "Mean % Truth: " <<
            std::accumulate(discrete_truths[i].begin(),
                discrete_truths[i].end(),
                static_cast<float>(0),
                [&](const float& a, const float& b) {
                    return a + b / discrete_truths[i].size();
                }) * 100 << "\n";
        for (size_t j = 0; j < discrete_truths[i].size(); ++j) {
            of << "Obstacle #" << j << "% detected: " << discrete_truths[i][j] * static_cast<float>(100) << "\n";
        }
    }
}//End print()
