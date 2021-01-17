#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <deque>

using namespace cv;
using namespace std;

class Display{
  private:
  Mat img{Mat(250, 400, CV_8UC3, Scalar(0,0,0))};
  string windowName;
  map<string, double> inputStats;


  void clearDisplay(){
    string inputText, statsText;
    int yValue = 20;

    for(auto &stats : inputStats){
      statsText = to_string(stats.second);
      inputText = stats.first + statsText;
      putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
      yValue += 20;
    }
  }

  public:
    Display(string in_windowName) : windowName(in_windowName) {
      namedWindow(windowName);
      imshow(windowName, img);
      waitKey(30);
    }
  
    void updateDisplay(map<string, double> inputs){
      clearDisplay();

      inputStats = inputs;
      string inputText, statsText;
      int yValue = 20;

      for(auto &stats : inputStats){
        statsText = to_string(stats.second);
        inputText = stats.first + statsText;
        putText(img, inputText, Point(5, yValue), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
        yValue += 20;
      }

      imshow(windowName, img);
      waitKey(1);
    }

    ~Display(){
      destroyWindow(windowName);
    }
};
