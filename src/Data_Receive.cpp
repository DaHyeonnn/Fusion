#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>

#include "Sensor_fusion/obj_msg.h"
#include "Sensor_fusion/LiDAR_BB.h"
#include "Sensor_fusion/LiDAR_BB_arr.h"
#include "Sensor_fusion/Camera_BB.h"
#include "Sensor_fusion/Camera_BB_arr.h"
#define PI 3.141592

using namespace std;

//함수 원형
void RCVD_LiDAR_DATA(const Sensor_fusion::obj_msg);
void RCVD_Camera_DATA_BB(const std_msgs::String);
void RCVD_Camera_DATA_img(const sensor_msgs::Image);
void Data_combine();
void final_fusion(const sensor_msgs::Image&, Sensor_fusion::LiDAR_BB_arr&, Sensor_fusion::Camera_BB_arr&);
void print_Camera_BB(cv::Mat&, vector<Sensor_fusion::Camera_BB>&);
void print_LiDAR_BB(cv::Mat&, vector<Sensor_fusion::LiDAR_BB>&);
bool check_overlap(Sensor_fusion::LiDAR_BB&, Sensor_fusion::Camera_BB&, float, float);
void trans_LiDAR_DATA(Sensor_fusion::LiDAR_BB_arr::Ptr);
void project_LiDAR_to_Img(vector<vector<float>>&, Sensor_fusion::LiDAR_BB_arr::Ptr);
void trans_Camera_DATA(Sensor_fusion::Camera_BB_arr::Ptr);
pair<pair<int, int>, pair<int, int>> s_to_minmax(string);
double s_to_f(string);
string f_to_s(float);
void set_calib_mat();

//final fusion용 구조체
struct Bbox{
    float xmin = 0;
    float xmax = 0;
    float ymin = 0;
    float ymax = 0;
    int flag = 0;
};

struct finalbox{
	float xmax = 0;
    float xmin = 0;
	float ymax = 0;
	float ymin = 0;
	float xcenter = 0;
	float ycenter = 0;
    float zcenter = 0;
	string Class;
};

//라이다 & 카메라 calibration용 matrix, calibration matrix setting
cv::Mat intrinsic_mat = cv::Mat(3,4,CV_32F, cv::Scalar::all(0));       // camera internal info
cv::Mat extrinsic_mat = cv::Mat(4,4,CV_32F, cv::Scalar::all(0));       // coord_trans
cv::Mat x_rotate_mat = cv::Mat(4,4,CV_32F, cv::Scalar::all(0)); 
cv::Mat final_project_mat = cv::Mat(3,4,CV_32F, cv::Scalar::all(0));   // project LiDAR to Camera matrix

//sync를 위한 전역변수
Sensor_fusion::obj_msg LiDAR_DATA_comb;
std_msgs::String Camera_DATA_comb;
sensor_msgs::Image Camera_IMG_DATA_comb;


void whole_processing(){
    //test
    Sensor_fusion::LiDAR_BB::Ptr test_Lidar1 (new Sensor_fusion::LiDAR_BB);
    Sensor_fusion::LiDAR_BB::Ptr test_Lidar2 (new Sensor_fusion::LiDAR_BB);
    Sensor_fusion::Camera_BB::Ptr test_Camera (new Sensor_fusion::Camera_BB);
    test_Lidar1->xmin_LiDAR = 170;
    test_Lidar1->xmax_LiDAR = 270;
    test_Lidar1->ymin_LiDAR = 210;
    test_Lidar1->ymax_LiDAR = 370;
    test_Lidar1->xcenter = 2.52;
    test_Lidar1->ycenter = 0.68;
    test_Lidar1->zcenter = 0.15;

    test_Lidar2->xmin_LiDAR = 320;
    test_Lidar2->xmax_LiDAR = 400;
    test_Lidar2->ymin_LiDAR = 90;
    test_Lidar2->ymax_LiDAR = 350;
    test_Lidar2->xcenter = 2.21;
    test_Lidar2->ycenter = -0.48;
    test_Lidar2->zcenter = -0.25;

    test_Camera->Class = "test";
    test_Camera->probability = 0.9;
    test_Camera->xmin_Camera = 30;
    test_Camera->xmax_Camera = 120;
    test_Camera->ymin_Camera = 40;
    test_Camera->ymax_Camera = 240;

    Sensor_fusion::LiDAR_BB_arr::Ptr test_Lidar_arr (new Sensor_fusion::LiDAR_BB_arr);
    Sensor_fusion::Camera_BB_arr::Ptr test_Camera_arr (new Sensor_fusion::Camera_BB_arr);
    test_Lidar_arr->LiDAR_BB_arr.push_back(*test_Lidar1);
    test_Lidar_arr->LiDAR_BB_arr.push_back(*test_Lidar2);
    test_Camera_arr->Camera_BB_arr.push_back(*test_Camera); 
    

    Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr(new Sensor_fusion::LiDAR_BB_arr);
    Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr(new Sensor_fusion::Camera_BB_arr);

    trans_LiDAR_DATA(LiDAR_BoundingBox_arr);
    trans_Camera_DATA(Camera_BoundingBox_arr);


    final_fusion(Camera_IMG_DATA_comb,*LiDAR_BoundingBox_arr,*Camera_BoundingBox_arr);

}

void RCVD_LiDAR_DATA(const Sensor_fusion::obj_msg LiDAR_DATA){
    //cout << "LiDAR msg : " << LiDAR_DATA << endl;
    for(int i=0;i<LiDAR_DATA.x.size();i++)
        cout << "LiDAR x : " << LiDAR_DATA.x[i] << "      LiDAR y : " << LiDAR_DATA.y[i] << "      LiDAR z : " << LiDAR_DATA.z[i] << endl;
    LiDAR_DATA_comb = LiDAR_DATA;
}

void RCVD_Camera_DATA_BB(const std_msgs::String Camera_DATA){
    //cout << "Camera_BB : " << Camera_DATA << endl;
    Camera_DATA_comb = Camera_DATA;
}

void RCVD_Camera_DATA_img(const sensor_msgs::Image Camera_DATA){
    Camera_IMG_DATA_comb = Camera_DATA; 
    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr = cv_bridge::toCvCopy(Camera_DATA ,"8UC3"); //sibal.......8UC3...
    // cv::Mat tmp = cv_ptr->image;
    // cv::imshow("sibal", tmp);
    // cv::waitKey(1);
    whole_processing(); //camera Img data rate로 sync 기준을 잡는다.
    
}

//-------------------------------------------라이다 데이터 변환 및 가공-------------------------------------------

void set_calib_mat(){
    //intrinsic mat setting
    intrinsic_mat.at<float>(0,0) = 371.288;
    intrinsic_mat.at<float>(0,1) = 0;
    intrinsic_mat.at<float>(0,2) = 312.874;
    intrinsic_mat.at<float>(0,3) = 0;

    intrinsic_mat.at<float>(1,0) = 0;
    intrinsic_mat.at<float>(1,1) = 371.612;
    intrinsic_mat.at<float>(1,2) = 241.294;
    intrinsic_mat.at<float>(1,3) = 0;

    intrinsic_mat.at<float>(2,0) = 0;
    intrinsic_mat.at<float>(2,1) = 0;
    intrinsic_mat.at<float>(2,2) = 1;
    intrinsic_mat.at<float>(2,3) = 0;

    //x_rotate_mat setting
    double theta = 20.5;
    x_rotate_mat.at<float>(0,0) = 1;
    x_rotate_mat.at<float>(0,1) = 0;
    x_rotate_mat.at<float>(0,2) = 0;
    x_rotate_mat.at<float>(0,3) = 0;

    x_rotate_mat.at<float>(1,0) = 0;
    x_rotate_mat.at<float>(1,1) = cos(theta*PI/180);
    x_rotate_mat.at<float>(1,2) = -sin(theta*PI/180);
    x_rotate_mat.at<float>(1,3) = 0;
    
    x_rotate_mat.at<float>(2,0) = 0;
    x_rotate_mat.at<float>(2,1) = sin(theta*PI/180);
    x_rotate_mat.at<float>(2,2) = cos(theta*PI/180);
    x_rotate_mat.at<float>(2,3) = 0;

    x_rotate_mat.at<float>(3,0) = 0;
    x_rotate_mat.at<float>(3,1) = 0;
    x_rotate_mat.at<float>(3,2) = 0;
    x_rotate_mat.at<float>(3,3) = 1;

    //extrinsic mat setting
    extrinsic_mat.at<float>(0,0) = 0;
    extrinsic_mat.at<float>(0,1) = -1;
    extrinsic_mat.at<float>(0,2) = 0;
    extrinsic_mat.at<float>(0,3) = 0.29;

    extrinsic_mat.at<float>(1,0) = 0;
    extrinsic_mat.at<float>(1,1) = 0;
    extrinsic_mat.at<float>(1,2) = -1;
    extrinsic_mat.at<float>(1,3) = 0.07;

    extrinsic_mat.at<float>(2,0) = 1;
    extrinsic_mat.at<float>(2,1) = 0;
    extrinsic_mat.at<float>(2,2) = 0;
    extrinsic_mat.at<float>(2,3) = 0;
    //additional
    extrinsic_mat.at<float>(3,0) = 0;
    extrinsic_mat.at<float>(3,1) = 0;
    extrinsic_mat.at<float>(3,2) = 0;
    extrinsic_mat.at<float>(3,3) = 1;
}

void calibration_LiDAR_to_Camera(){
    final_project_mat = intrinsic_mat * x_rotate_mat * extrinsic_mat;
}

void project_LiDAR_to_Img(vector<vector<float>>& OBJ, Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr){
    cv::Mat LiDAR3DPoint(4,1,CV_32F, cv::Scalar(1,0)); //(XL, YL, ZL, 1)
    cv::Mat Img2DPoint(3,1,CV_32F); //(XC/ZC, YC/ZC, 1)
    //무조건 가까운 x를 기준으로 Bbox를 형성하자. 깊이가 깊은(x가 뒤쪽)상황에선 에러가 나는데 그건 뒤쪽 기준 비박스 형성하고 앞쪽 기준 비박스 형성해서 서로 합치면 될듯
    //일단은 앞을(x가 최소인 지점)기준으로 Bbox형성을 먼저 해보자. 
    //한빈아.... 그 일단 욕심부리지말고 비박스투영먼저 해보자. 실제좌표는 나중에 받아와도 대자나..........시발 인생 ㅈ같네..
    //나중에는 모든 minmax경우의수로 발생하는 비박스를 다 만들고 최종적으로 합치면 될듯.
    //LiDAR_BB-xmin = LiDAR_Point-ymin, LiDAR_BB-xmax = LiDAR_Point-ymax, LiDAR_BB-ymin = LiDAR_Point-
    for(int i = 0; i < OBJ.size(); i++){
        Sensor_fusion::LiDAR_BB::Ptr tmp_Lidar (new Sensor_fusion::LiDAR_BB);
        //x,y,z값은 라이다 좌표 그대로 보존해야 한다.
        tmp_Lidar->xcenter = OBJ[i][0]; //x
        tmp_Lidar->ycenter = OBJ[i][1]; //y
        tmp_Lidar->zcenter = OBJ[i][2]; //z


        //y값 회전변환 고려를 안해서 일단 y에 마이너스를 붙히자, 
        //-----------------각 픽셀의 min값을 구해보자.
        LiDAR3DPoint.at<float>(0,0) = OBJ[i][3]; //xMin, 가까운 x를 기준으로...
        LiDAR3DPoint.at<float>(1,0) = OBJ[i][5]; //yMin
        LiDAR3DPoint.at<float>(2,0) = OBJ[i][7]; //zMin

        Img2DPoint = final_project_mat * LiDAR3DPoint;

        //일단 스케일 펙터가 이상해서 뺐다.
        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);

        //이렇게 나온 두 값이 ymin과 zmin을 변환한 것으로 //xmin과 ymin이 되어야 하는데 회전 변환이 안됐다면 xmin과 ymax가 될 것이다.
        tmp_Lidar->xmin_LiDAR = Img2DPoint.at<float>(0,0);  //xmin
        tmp_Lidar->ymin_LiDAR = -Img2DPoint.at<float>(1,0); //ymin

        //-----------------이제 max값을 구해보자.
        LiDAR3DPoint.at<float>(0,0) = OBJ[i][3]; //xMin, 가까운 x를 기준으로...
        LiDAR3DPoint.at<float>(1,0) = OBJ[i][6]; //yMax
        LiDAR3DPoint.at<float>(2,0) = OBJ[i][8]; //zMax        

        Img2DPoint = final_project_mat * LiDAR3DPoint;

        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);

        tmp_Lidar->xmax_LiDAR = Img2DPoint.at<float>(0,0);  //xmax
        tmp_Lidar->ymax_LiDAR = -Img2DPoint.at<float>(1,0); //ymax

        //그리고 ymin과 ymax자리를 바꿔보자.. 회전변환 때문에
        float tmp = tmp_Lidar->xmax_LiDAR;
        tmp_Lidar->xmax_LiDAR = tmp_Lidar->xmin_LiDAR;
        tmp_Lidar->xmin_LiDAR = tmp;
        tmp = tmp_Lidar->ymax_LiDAR;
        tmp_Lidar->ymax_LiDAR = tmp_Lidar->ymin_LiDAR;
        tmp_Lidar->ymin_LiDAR = tmp;
        tmp_Lidar->ymax_LiDAR = abs(tmp_Lidar->ymax_LiDAR);
        tmp_Lidar->ymin_LiDAR = abs(tmp_Lidar->ymin_LiDAR);

        LiDAR_BoundingBox_arr->LiDAR_BB_arr.push_back(*tmp_Lidar);  
        //cout<<"scale factor : "<<Img2DPoint.at<float>(2,0)<<endl;//이게 왜 마이너스가 나오냐?
        cout<<"LiDAR pixel : ";
        cout<<"xmin: "<<tmp_Lidar->xmin_LiDAR <<",  xmax: "<<tmp_Lidar->xmax_LiDAR <<",  ymin: "<<tmp_Lidar->ymin_LiDAR <<",  ymax: "<<tmp_Lidar->ymax_LiDAR<<endl; 
    }

}

void trans_LiDAR_DATA(Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr){
    Sensor_fusion::obj_msg tmp_LiDAR = LiDAR_DATA_comb; //data LOCK process... 중간의 데이터 변화를 막는 과정

    //OBJ msg trans
    vector<vector<float>> OBJ;
    vector<float> ONE_OBJ; //x,y,z,xmin,xmax,ymin,ymax,zmin,zmax
    ONE_OBJ.resize(9);
    for(int i=0;i<tmp_LiDAR.x.size();i++){
        ONE_OBJ[0] = tmp_LiDAR.x[i];
        ONE_OBJ[1] = tmp_LiDAR.y[i];
        ONE_OBJ[2] = tmp_LiDAR.z[i];
        ONE_OBJ[3] = tmp_LiDAR.xMin[i];
        ONE_OBJ[4] = tmp_LiDAR.xMax[i];
        ONE_OBJ[5] = tmp_LiDAR.yMin[i];
        ONE_OBJ[6] = tmp_LiDAR.yMax[i];
        ONE_OBJ[7] = tmp_LiDAR.zMin[i];
        ONE_OBJ[8] = tmp_LiDAR.zMax[i];
        OBJ.push_back(ONE_OBJ);
    }

    //라이다 BB가 겹쳐있을 경우 해결해 줘야함.
    set_calib_mat();
    calibration_LiDAR_to_Camera();
    project_LiDAR_to_Img(OBJ,LiDAR_BoundingBox_arr);

}

//-------------------------------------------카메라 데이터 변환 및 가공-------------------------------------------

double s_to_f(string st){
    double tmp = 0;
    string tmp_st = st.substr(2,2);
    int tmp_int = stoi(tmp_st);
    tmp = (double)tmp_int / 100;
    return tmp;
}

pair<pair<int, int>, pair<int, int>> s_to_minmax(string st) {
    pair<pair<int, int>, pair<int, int>> tmp;
    int tmp_xmin = 0, tmp_xmax = 0, tmp_ymin = 0, tmp_ymax = 0;
    int srt = 0;
    int flag = 1; //xmin = 1, xmax = 2, ymin = 3, ymax = 4
    bool check_srt = 0;//숫자가 시작하는 시점인지 체크해야함

    for (int i = 0; i < st.size(); i++) {
        if (st[i] != '.') continue;
        if(flag==1){
            tmp_xmin = stoi(st.substr(srt, srt - i));
        }
        else if (flag == 2) {
            tmp_ymin = stoi(st.substr(srt, srt - i));
        }
        else if (flag == 3) {
            tmp_xmax = stoi(st.substr(srt, srt - i));
        }
        else {
            tmp_ymax = stoi(st.substr(srt, srt - i));
            break;
        }
        i += 3;
        flag++;
        srt = i;
    }

    tmp = make_pair(make_pair(tmp_xmin, tmp_xmax), make_pair(tmp_ymin, tmp_ymax));
    return tmp;
}


void trans_Camera_DATA(Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr){
    std_msgs::String tmp_Camera = Camera_DATA_comb; //data LOCK process... 중간의 데이터 변화를 막는 과정
    string Camera_obj = tmp_Camera.data;
    vector<string> OBJ;
    int tmp = 0;

    for(int i = 0;i < Camera_obj.size(); i++){
        if(Camera_obj[i] != '/') continue;
        string tmp_st = Camera_obj.substr(tmp, i - tmp);
        OBJ.push_back(tmp_st + '-'); //코드 편의를 위해 끝에 - 추가
        tmp = i + 1;
    }

    for(int i = 0;i < OBJ.size(); i++){
        Sensor_fusion::Camera_BB::Ptr tmp_Camera_Bbox (new Sensor_fusion::Camera_BB);
        int flag = 1; // 1 = class, 2 = minmax, 3 = probability
        tmp = 0;

        for(int j = 0; j < OBJ[i].size(); j++){
            if(OBJ[i][j] != '-') continue;

            string tmp_st;
            if(flag == 1){//calss
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                tmp_Camera_Bbox->Class = tmp_st;
            }
            else if(flag == 2){//minmax
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                pair<pair<int,int>,pair<int,int>> minmax = s_to_minmax(tmp_st);
                //xmin, xmax, ymin, ymax
                tmp_Camera_Bbox->xmin_Camera = minmax.first.first;
                tmp_Camera_Bbox->xmax_Camera = minmax.first.second;
                tmp_Camera_Bbox->ymin_Camera = minmax.second.first;
                tmp_Camera_Bbox->ymax_Camera = minmax.second.second;
            }
            else{//probability
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                tmp_Camera_Bbox->probability = s_to_f(tmp_st);
            }
            tmp = j + 1;
            flag++;
        }
        Camera_BoundingBox_arr->Camera_BB_arr.push_back(*tmp_Camera_Bbox);
    }
    //test print
    // cout << "<Camera OBJ>" << endl;
    // for(int i=0;i<Camera_BoundingBox_arr->Camera_BB_arr.size();i++){
    //     cout<<Camera_BoundingBox_arr->Camera_BB_arr[i].Class<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].xmin_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].xmax_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].ymin_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].ymax_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].probability<<endl;
    // }
}


//-------------------------------------------마지막 퓨전하는 곳-------------------------------------

string f_to_s(float tmp){
    string st;
    int tmp1 = tmp; //정수부
    int tmp2 = (tmp - tmp1) * 100; //소수부
    st = to_string(tmp1)+"."+to_string(tmp2);
    return st;
}

void print_LiDAR_BB(cv::Mat& fin_Image,vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox){
    for(int i=0;i<LiDAR_Bbox.size();++i){
        float width = LiDAR_Bbox[i].xmax_LiDAR- LiDAR_Bbox[i].xmin_LiDAR;
        float height =  LiDAR_Bbox[i].ymax_LiDAR- LiDAR_Bbox[i].ymin_LiDAR;               
        cv::rectangle(fin_Image,cv::Rect(LiDAR_Bbox[i].xmin_LiDAR,LiDAR_Bbox[i].ymin_LiDAR,width,height),cv::Scalar(255,0,0),2,1,0);
    }
}

void print_Camera_BB(cv::Mat& fin_Image,vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    for(int i=0;i<Camera_Bbox.size();++i){
        if(Camera_Bbox[i].probability < 0.4) continue;
        float width = Camera_Bbox[i].xmax_Camera- Camera_Bbox[i].xmin_Camera;
        float height =  Camera_Bbox[i].ymax_Camera- Camera_Bbox[i].ymin_Camera;
        cv::rectangle(fin_Image,cv::Rect(Camera_Bbox[i].xmin_Camera,Camera_Bbox[i].ymin_Camera,width,height),cv::Scalar(0,0,255),2,1,0);    
    }
}

bool check_overlap(Sensor_fusion::LiDAR_BB& LiDARBB,Sensor_fusion::Camera_BB& CameraBB, float width_C, float height_C){
    bool maxx = (CameraBB.xmax_Camera - LiDARBB.xmax_LiDAR < width_C / 2) && (CameraBB.xmax_Camera - LiDARBB.xmax_LiDAR > -width_C / 2);
    bool minx = (CameraBB.xmin_Camera - LiDARBB.xmin_LiDAR < width_C / 2) && (CameraBB.xmin_Camera - LiDARBB.xmin_LiDAR > -width_C / 2);
    bool miny = (CameraBB.ymin_Camera - LiDARBB.ymin_LiDAR < height_C / 2) && (CameraBB.ymin_Camera - LiDARBB.ymin_LiDAR > -height_C / 2);
    bool maxy = (CameraBB.ymax_Camera - LiDARBB.ymax_LiDAR < height_C / 2) && (CameraBB.ymax_Camera - LiDARBB.ymax_LiDAR > -height_C / 2);
    return maxx && minx && miny && maxy;
}

void final_fusion(const sensor_msgs::Image& Img, Sensor_fusion::LiDAR_BB_arr& LiDARBB, Sensor_fusion::Camera_BB_arr& CameraBB){
    vector<Sensor_fusion::LiDAR_BB> LiDAR_Bbox(LiDARBB.LiDAR_BB_arr);
    vector<Sensor_fusion::Camera_BB> Camera_Bbox(CameraBB.Camera_BB_arr);
    cv::Mat fin_Image;
    cv_bridge::CvImagePtr cv_ptr;
    bool switch_LiDAR_BB = 1, switch_Camera_BB = 1, switch_Fusion_BB = 1;

    cv_ptr = cv_bridge::toCvCopy(Img, "8UC3");
    fin_Image= cv_ptr->image;

    if(switch_LiDAR_BB) print_LiDAR_BB(fin_Image,LiDAR_Bbox);
    if(switch_Camera_BB) print_Camera_BB(fin_Image,Camera_Bbox);

    finalbox fusionbox; //퓨전은 카메라 기준으로 진행한다. 카메라에 라이다 Bbox를 매칭시키자.
    for(int i = 0;i < Camera_Bbox.size(); i++){
        if(Camera_Bbox[i].probability < 0.4) continue; //굳이 한번 더?
        float width_C = Camera_Bbox[i].xmax_Camera - Camera_Bbox[i].xmin_Camera;
        float height_C =  Camera_Bbox[i].ymax_Camera - Camera_Bbox[i].ymin_Camera;
	  	
        for(int j = 0;j < LiDAR_Bbox.size(); j++){
            float width_L = LiDAR_Bbox[j].xmax_LiDAR - LiDAR_Bbox[j].xmin_LiDAR;
            float height_L =  LiDAR_Bbox[j].ymax_LiDAR - LiDAR_Bbox[j].ymin_LiDAR;     

            if(check_overlap(LiDAR_Bbox[j],Camera_Bbox[i],width_C,height_C)){
                fusionbox.xmax = (Camera_Bbox[i].xmax_Camera + LiDAR_Bbox[j].xmax_LiDAR) / 2;
                fusionbox.ymax = (Camera_Bbox[i].ymax_Camera + LiDAR_Bbox[j].ymax_LiDAR) / 2;
                fusionbox.xmin = (Camera_Bbox[i].xmin_Camera + LiDAR_Bbox[j].xmin_LiDAR) / 2;
                fusionbox.ymin = (Camera_Bbox[i].ymin_Camera + LiDAR_Bbox[j].ymin_LiDAR) / 2;
                fusionbox.xcenter = LiDAR_Bbox[j].xcenter;
                fusionbox.ycenter = LiDAR_Bbox[j].ycenter;
                fusionbox.Class = Camera_Bbox[i].Class;
                fusionbox.zcenter = LiDAR_Bbox[j].zcenter;
                //fusionbox.probability = Camera_Bbox[i].probability;

                float width_F = fusionbox.xmax- fusionbox.xmin;
                float height_F=  fusionbox.ymax- fusionbox.ymin;
                cv::rectangle(fin_Image,cv::Rect(fusionbox.xmin,fusionbox.ymin,width_F,height_F),cv::Scalar(0,255,255),3,1,0);

                string text = fusionbox.Class+"-"+"x:"+f_to_s(fusionbox.xcenter)+ " "+"y:"+ f_to_s(fusionbox.ycenter)+ " "+"z:"+ f_to_s(fusionbox.zcenter);
                int font_face = cv::FONT_HERSHEY_SIMPLEX; //폰트 종류
                double font_scale = 0.7; //폰트 크기
                int thickness = 2;
                int baseline;
                cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
                cv::Point origin;
                origin.x = fusionbox.xmin;// - text_size.width / 2;  
                origin.y = fusionbox.ymin - text_size.height / 2;
                cv::putText(fin_Image, text, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness, 8, 0); 
                break;//중복처리 과정이 따로 없이 바로브레이크 박음.. 이거 나중에 고쳐야함
            }
		}	
	}

    // sensor_msgs::ImagePtr pubImg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", fin_Image).toImageMsg();
    // pub1.publish(pubImg);
    
    cv::imshow("fusion", fin_Image);
    cv::waitKey(1);
    fin_Image.release();
	Camera_Bbox.clear();
    LiDAR_Bbox.clear();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "Receive_fusion_data"); //node name 
	ros::NodeHandle nh_L;         //nodehandle,LiDAR
    ros::NodeHandle nh_C;         //nodehandle,Camera
    ros::NodeHandle nh_I;         //nodehandle,Image

    ros::Subscriber sub_LiDAR = nh_L.subscribe<Sensor_fusion::obj_msg> ("/Lidar_obj", 10, RCVD_LiDAR_DATA);
    ros::Subscriber sub_Camera_BB = nh_C.subscribe<std_msgs::String> ("/main/classes", 10, RCVD_Camera_DATA_BB);
    ros::Subscriber sub_Camera_img = nh_I.subscribe<sensor_msgs::Image> ("/main/img", 10, RCVD_Camera_DATA_img); //카메라에 라이다 인풋 속도 맞추기로 가정
    
    ros::spin();
}