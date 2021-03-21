#include <string>
#include <fstream>
#include <sl/Camera.hpp>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "pcl.hpp"

using namespace std;

class Recorder {
    public:
        Recorder() : frameNum(0){

        }

        Recorder(string dir) : dir(dir), frameNum(0) {
            mkdir(dir.c_str(), 0777);
        }

        void open(string dir) {
            this->dir = dir;
            mkdir(dir.c_str(), 0777);
        }

        void writeFrame(sl::Mat &frame) {
            ofstream fout(dir+ "/pc" + to_string(frameNum) + ".pc");
            sl::Mat cpuFrame; 
            frame.copyTo(cpuFrame, sl::COPY_TYPE::GPU_CPU);
            for(int r = 0; r < cpuFrame.getHeight(); r++) {
                for(int c = 0; c < cpuFrame.getWidth(); c++) {
                    sl::float4 val;
                    cpuFrame.getValue(c, r, &val);
                    fout << val.x << " " << val.y << " " << val.z << " " << val.w << "\n";
                }
            }
            //fout << endl;
            fout.close();
            frameNum++;
            std::cout << frameNum << endl;
        }

        

    private:
        int frameNum;
        string dir;

};

class Reader {
    public:
        Reader() : dir("") {
        }
        
        void open(string dir) {	
            this->dir = dir;
            map<int, string> fData;

            //Long-winded directory opening (no glob, sad)
            DIR * pcd_dir;
            pcd_dir = opendir(dir.c_str());
            if (NULL == pcd_dir) std::cerr<<"Input folder not exist\n";    
            
            struct dirent *dp = NULL;
            do {
                if ((dp = readdir(pcd_dir)) != NULL) {
                    std::string file_name(dp->d_name);
                    // the lengh of the tail str is at least 4
                    if (file_name.size() < 5) continue; //make it 5 to get the single digit
                    std::cout<<"file_name is "<<file_name<<std::endl;

                    pcd_names.push_back(file_name);
                    
                    int s = file_name.find_first_of("0123456789");
                    int l = file_name.find(".");
                    string keyS = file_name.substr(s,l-s );
                    int key = stoi(keyS);
                    //cout << key << endl;
                    fData[key] = file_name;
            
                }
            } while (dp != NULL);
            std::sort(pcd_names.begin(), pcd_names.end());

            pcd_names.clear();
            for(auto i : fData) {
                pcd_names.push_back(i.second);
                cout << "reader says: " <<  i.second << endl;
            }
	    }
	
        void load(int i, sl::Mat &zed, bool pcl) {
            std::string pcd_name = pcd_names[i];
 	        std::string full_path = dir + std::string("/") + pcd_name;
            
            if(pcl) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
                loadPCD(pc_pcl, full_path);
                PclToZed(zed, pc_pcl);
            } else {
                ifstream fin(full_path);
                
                sl::Resolution r = zed.getResolution();
                int width = r.width;

                std::string xs, ys, zs, cs;
                int q = 0;
                while(fin >> xs >> ys >> zs >> cs) {

                    float px = std::stof(xs);
                    float py = std::stof(ys);
                    float pz = std::stof(zs);
                    float pw = std::stof(cs);

                    if(isnan(pw) || !isfinite(pw) || isnan(px) || !isfinite(px) ) pw = 0.0;
                    //cout << "color: " << pw << endl;

                    zed.setValue(q%width, q/width, sl::float4(px, py, pz, pw));
                    q++;

                }
                zed.updateGPUfromCPU();
            }
            
        }
        

    private:
        string dir;
        std::vector<std::string> pcd_names;
};