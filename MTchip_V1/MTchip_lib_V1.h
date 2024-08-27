#pragma once
#pragma once

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp> //mophorlogical operation
#include<opencv2/core.hpp>
#include <chrono>
#include <fstream>

using namespace cv;
using namespace std;

typedef struct
{
	int interval[4];
	int xpitch[3]; //in unit of pixel 
	int ypitch[3];
	int carx;
	int cary;
}SettingP_;


typedef struct
{
	double TDwidth;
	double TDmaxW;
	double TDminW;
	double TDheight;
	double TDmaxH;
	double TDminH;

}sizeTD_;

typedef struct
{
	int thresmode; //0:gray ; 1:RGB ; 2:HSV ;3:AutoUP ; 4AutoDown
	int bgmax[3];
	int bgmin[3];
	int fgmax[3];
	int fgmin[3];

}thresP_;

typedef struct
{
	int PICmode;
	int Outputmode;
	int imgcols;
	int imgrows;
	double correctTheta;
}ImgP_;

/*general operation*/
Point find_piccenter(Mat src);
Mat CropIMG(Mat img, Rect size);


int findBoundary(Mat creteriaIMG, Rect inirect, char direction);
std::tuple<Rect, Point>FindMaxInnerRect(Mat src, Mat colorSRC, sizeTD_ target, Point TDcenter);
Mat RotatecorrectImg(double Rtheta, Mat src);


/******Single- phase chip:::*******/
//version 3
std::tuple<int, Mat, Point, Mat>Uchip_singlephaseDownV3(int flag, Mat stIMG, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point2f creteriaPoint, Point IMGoffset, ImgP_ imageParm);


/******Dual- phase hcip:::********/
std::tuple<int, Mat, Point, Mat>Uchip_dualphase(int flag, Mat stIMG, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point2f creteriaPoint, Point IMGoffset, ImgP_ imageParm);

void PairChip_Finder(int& flag, Mat imgInput, Mat& imgThres, Mat& imgOut, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, vector<Point2f>& pairCentre, Point IMGoffset, ImgP_ imageParm);
