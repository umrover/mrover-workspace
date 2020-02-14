#include "perception.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>

using namespace cv;
using namespace std;

//Function that writes depths to a file given a Matrix with int type values
void writeDepths(Mat &depthIn, ofstream &depth)
{
    //Get size
    int cols = depthIn.cols;
    int rows = depthIn.rows;

    //Output cols and rows
    cerr<<cols<<endl;
    cerr<<rows<<endl;
    
    //Write depth values to depth file stream
    for(int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            depth<<depthIn.at<int>(y,x)<<" ";
        }
        depth<<endl<<endl;
    }

    depth.close();
}

//Does calcualations to find normalization multiplier for assigning disparity matrix values
float* calculateMultiplier(float max_depth, float step)
{
    float out[2];
    float columns = max_depth/step;
    float multiplier = columns/max_depth;
    cerr<<columns<<endl;
    cerr<<multiplier<<endl;
    out[0] = columns;
    out[1] = multiplier;
    float* end = &(out[0]);
    return end;
}

//Calculates vDisparity Map
Mat vDisparity(Mat &depthIn)
{
    patchNaNs(depthIn, 0.0);
    depthIn = max(depthIn, 0.7);
    depthIn = min(depthIn, 10.0);

    //Get size
    int cols = depthIn.cols;
    int rows = depthIn.rows;
    
    //Calculate multiplier and column size for maps
    float* temp = calculateMultiplier(9.3, 0.12);
    float imgCols = *temp;
    float multiplier = *(temp+1);
    cerr<<"Image Columns: "<<imgCols<<endl;
    cerr<<"Multiplier: "<<multiplier<<endl;

    //Create empty vMap
    Mat vMap(rows, imgCols, CV_32SC1, Scalar(0));
    Mat white(rows, imgCols, CV_32SC1, Scalar(255));
    Mat white1(rows,imgCols, CV_32FC1, Scalar(0));

    //Traverse through image and increment values in vMap that correspond with depth
    for(int y = 0; y < rows; ++y)
    {
    
        for(int x = 0; x < cols; ++x)
        {
            int column = int((depthIn.at<float>(y, x)-0.7)*multiplier);
            vMap.at<int>(y, column) = vMap.at<int>(y, column)+1;
        }

    }

    //Write and show raw vDisparity Mat
    ofstream vMapS;
    vMapS.open("vMap.txt");
    imshow("vMap", vMap);
    writeDepths(vMap,vMapS);

    //Write and show normalized vDisparity Mat
    Mat normalized(720,imgCols,CV_32FC1);
    normalize(vMap, normalized, 255, 0.0, NORM_MINMAX);
    ofstream normal;
    normal.open("Noramlized.txt");
    writeDepths(normalized, normal);

    //Theoretically multiply all values by 20
    normalized.convertTo(normalized,-1,20,0);

    //Write out rows and cols for debugging
    cout<<rows<<endl;
    cout<<cols<<endl;

    waitKey();
    return vMap;

}







































/*
//Create file stream
    ofstream depth;
    ofstream v;
    v.open("vMap.txt");
    depth.open("outputDepth1.txt");

    patchNaNs(depthIn, 0.0);
    
    for(int y = 0; i < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            v<<depthIn.at<float>(y,x)<<" ";
        }
        v<<endl<<endl;
    }
   
    for(int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            depth<<depthIn.at<double>(i,j)<<" ";
        }
        depth<<endl<<endl;
    }
    
    
    depth.close();
    v.close();

        
    //Turn super small values to zeros and divide all values by 1000
    for(int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if(depthIn.at<double>(j,i) > 0.09)
            {
                depthIn.at<double>(j,i) = 9;
            }
            //else if(depthIn.at<double>(i,j)<.00700)
              //  depthIn.at<double>(i,j) = 0.7;
            
            else if(depthIn.at<double>(j,i) < 0.09)
            {
                depthIn.at<double>(j,i) = depthIn.at<double>(j,i)*100;
            }
        }
    }
    

    patchNaNs(depthIn, 0.0);
    depthIn = max(depthIn, 0.7);
    depthIn = min(depthIn, 10.0);
    
*/