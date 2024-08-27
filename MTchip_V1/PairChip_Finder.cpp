#include "MTchip_lib_V1.h"
#include "OpenCV_Extension_Tool.h"


//bool bSort(BlobInfo a, BlobInfo b,Point pt) { return (i < j); }


void PairChip_Finder(int& flag, Mat imgInput,Mat& imgThres,Mat& imgOut, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, vector<Point2f>& pairCentre, Point IMGoffset, ImgP_ imageParm)
{
	flag = 0;

	//----Threshold 沿用 Uchip_singlephaseDownV3
#pragma region 多於一個地方呼叫相同程式碼 應將此段變成一個函式
	imgThres = Mat::zeros(imgInput.rows, imgInput.cols, CV_8UC1);
	Point crossCenter;
	Mat thresRot;
	Point crossCenternew;
	imgOut = imgInput.clone();
	//Thres parameters:::
	Mat Gimg, gauGimh;
	Mat gauBGR;
	vector<vector<Point>>  contH, contRot;
	vector<Vec4i> hierH, hierRot;
	vector<vector<Point>> reqConH;

	Mat adptThres;
	Mat HIGHthres;
	int adaptWsize = 3;
	int adaptKsize = 2;

	vector<vector<Point>>  contours, REQcont;
	Rect retCOMP;
	vector<Rect> Rectlist;
	vector<Point2f> center;
	vector<double> distance;
	Point2f piccenter;
	int minIndex;
	vector<Point> approx;
	vector< double> approxList;
	double areacomthres;


	//Step.1  pre-processing to enhance contrast
	imgInput.convertTo(imgInput, -1, 1.2, 0);
	cv::GaussianBlur(imgInput, gauBGR, Size(0, 0), 13);
	cv::addWeighted(imgInput, 1.5, gauBGR, -0.7, 0.0, imgInput);

	//Step.2  pre-processing of denoise
	cv::cvtColor(imgInput, Gimg, COLOR_RGB2GRAY);
	cv::fastNlMeansDenoising(Gimg, gauGimh, 3, 7, 21);

	//Step.3 threshold filtering
	if (thresParm.thresmode == 0)
	{
		Scalar maxthres = Scalar(thresParm.fgmax[0], thresParm.fgmax[0], thresParm.fgmax[0]);
		Scalar minthres = Scalar(thresParm.fgmin[0], thresParm.fgmin[0], thresParm.fgmin[0]);
		cv::inRange(gauGimh, minthres, maxthres, HIGHthres);
		cv::medianBlur(HIGHthres, imgThres, 17);
		Mat Kcomclose = Mat::ones(Size(5, 5), CV_8UC1);  //Size(10,5)
		cv::morphologyEx(imgThres, imgThres, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 1);//1 //2
	}
	else// thresParm.thresmode==3 & 4
	{
		int nThresholdType = THRESH_BINARY_INV;

		if (thresParm.thresmode == 4)
			nThresholdType = THRESH_BINARY;

		if (thresParm.bgmax[0] & 1)
		{
			adaptWsize = thresParm.bgmax[0];
			adaptKsize = thresParm.fgmax[0];
		}
		else
		{
			adaptWsize = thresParm.bgmax[0] + 1;
			adaptKsize = thresParm.fgmax[0];
		}
		adaptiveThreshold(gauGimh, adptThres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, nThresholdType, adaptWsize, adaptKsize);//55,1 //ADAPTIVE_THRESH_MEAN_C

		cv::medianBlur(adptThres, imgThres, 7);
		Mat Kcomclose = Mat::ones(Size(5, 5), CV_8UC1);  //Size(10,5)
		cv::morphologyEx(imgThres, imgThres, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 1);//1 //2
	}
#pragma endregion

	//output parameters:::


	int maxArea= target.TDheight*target.TDwidth*target.TDmaxW* target.TDmaxH;
	int minArea= target.TDheight * target.TDwidth * target.TDminW* target.TDminH;
	//---邊緣進行切割
	vector<BlobInfo> vRegions = RegionPartition(imgThres, maxArea, minArea);

	//----DebugImg
	Mat Debug = Mat::zeros(imgInput.rows, imgInput.cols, CV_8UC1);

	//vector<vector<Point>> vPt;

	//for (int i = 0; i < vRegions.size(); i++)
	//	vPt.push_back(vRegions[i].Points());

	//drawContours(Debug, vPt, -1, Scalar(255, 255, 255));


	Point2f ptC = Point2f(imgInput.cols / 2,imgInput.rows / 2);

	//std::sort(myvector.begin() + 4, myvector.end(),, myfunction);

	std::sort(vRegions.begin(), vRegions.end(), [&,ptC](BlobInfo& a, BlobInfo& b)
		{
			norm(a.Center() - ptC);			
			return norm(a.Center() - ptC)< norm(b.Center() - ptC);
		});


	vector<vector<Point>> vPtC_Chip;

	vector<Point> vPtC_Rect;

	for (int i = 0; i < 2; i++)
		vPtC_Chip.push_back(vRegions[i].contour());

	drawContours(Debug, vPtC_Chip, -1, Scalar(255, 255, 255));

	findNonZero(Debug, vPtC_Rect);

	cv::RotatedRect min_rect = minAreaRect(vPtC_Rect);

	Point2f vertices[4];
	min_rect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(imgOut, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 5);

	drawMarker(imgOut, min_rect.center, Scalar(0, 0, 255),5);
	circle(imgOut, ptC,15, Scalar(0, 255, 255),1);
	flag = 9;
}