#include "MTchip_lib_V1.h"
#include "OpenCV_Extension_Tool.h"


//bool bSort(BlobInfo a, BlobInfo b,Point pt) { return (i < j); }


void PairChip_Finder(int& flag, Mat imgInput,Mat& imgThres,Mat& imgOut, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point2f& creteriaPoint, Point IMGoffset, ImgP_ imageParm)
{
	flag = 0;

	imgOut = imgInput.clone();

	funcThreshold(imgInput, imgThres, thresParm,imageParm);

	int maxArea= target.TDheight*target.TDwidth*target.TDmaxW* target.TDmaxH;
	int minArea= target.TDheight * target.TDwidth * target.TDminW* target.TDminH;
	//---邊緣進行切割
	vector<BlobInfo> vRegions = RegionPartition(imgThres, maxArea, 0);

	if (vRegions.size() == 0)
	{
		flag = 1;
		throw "something wrong::threshold value issue";
	}


	vector<BlobInfo> vRegions_Filtered;
	//--依照條件過濾

	for (int i = 0; i < vRegions.size(); i++)
	{
		int ww = vRegions[i].Xmax() - vRegions[i].Xmin();
		int hh = vRegions[i].Ymax() - vRegions[i].Ymin();

		if (ww > target.TDwidth * target.TDmaxW || ww < target.TDwidth * target.TDminW)
			continue;

		if (hh> target.TDheight * target.TDmaxH || hh < target.TDheight * target.TDminH)
			continue;

		vRegions_Filtered.push_back(vRegions[i]);

		fillConvexPoly(imgThres, vRegions[i].Points(), Scalar(255, 255, 255));
		
		rectangle(imgThres, Point(vRegions[i].Xmin(), vRegions[i].Ymin()), Point(vRegions[i].Xmax(), vRegions[i].Ymax()), Scalar(255, 255, 255), cv::FILLED);
		rectangle(imgOut, Point(vRegions[i].Xmin(), vRegions[i].Ymin()), Point(vRegions[i].Xmax(), vRegions[i].Ymax()), Scalar(255,255,255),5);
	}

	if (vRegions_Filtered.size() <2 )
	{
		flag = 2;
		throw "something wrong::potential object doesn't fit suitable dimension";
	}

	//----DebugImg
	Mat Debug = Mat::zeros(imgInput.rows, imgInput.cols, CV_8UC1);
	Point2f ptC = Point2f(imgInput.cols / 2,imgInput.rows / 2);

	std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&,ptC](BlobInfo& a, BlobInfo& b)
		{
			norm(a.Center() - ptC);			
			return norm(a.Center() - ptC) < norm(b.Center() - ptC);
		});

	vector<vector<Point>> vPtC_Chip;

	vector<Point> vPtC_Rect;

	for (int i = 0; i < 2; i++)
		vPtC_Chip.push_back(vRegions_Filtered[i].contour());

	drawContours(Debug, vPtC_Chip, -1, Scalar(255, 255, 255));

	findNonZero(Debug, vPtC_Rect);

	cv::RotatedRect min_rect = minAreaRect(vPtC_Rect);

	cv::Point vertices[4];
	Point2f vertices2f[4];

	min_rect.points(vertices2f);

	for (int i = 0; i < 4; i++)
		line(imgOut, vertices2f[i], vertices2f[(i + 1) % 4], Scalar(0, 255, 0), 3);

	for (int i = 0; i < 4; ++i)
		vertices[i] = vertices2f[i];

	//影像中心
	cv::circle(imgOut,ptC,9,Scalar(0, 255, 255),FILLED,LINE_AA);

	// Chip Pair 中心
	cv::circle(imgOut,min_rect.center,6,Scalar(255, 0, 255),FILLED,LINE_AA);
	cv::line(imgOut, Point(0, min_rect.center.y), Point(imgOut.size[1], min_rect.center.y), Scalar(255, 255, 255), 1, 8);
	cv::line(imgOut, Point(min_rect.center.x, 0), Point(min_rect.center.x, imgOut.size[0]), Scalar(255, 255, 255), 1, 8);

	//-----計算是否偏離

	if (chipsetting.xpitch[0] > 0)
		if (abs(ptC.x - min_rect.center.x) > chipsetting.xpitch[0])
			flag = 6; //畫面中心無晶片

	if(chipsetting.ypitch[0]>0)
		if (abs(ptC.y - min_rect.center.y) > chipsetting.ypitch[0])
			flag = 6; //畫面中心無晶片

	if (chipsetting.ypitch[0] <= 0 & chipsetting.xpitch[0] <= 0)
	{
		cv::fillConvexPoly(Debug, vertices, 4, Scalar(255, 255, 255));
		uchar uCenter = Debug.at<uchar>(ptC.y, ptC.x);

		if (uCenter == 0)
			flag = 6; //畫面中心無晶片
	}

	if (flag == 6)
	{
		throw "something wrong::potential object doesn't fit suitable dimension";
	}

	
	if (imageParm.correctTheta != 0) //平台不能轉的case 
	{
		Mat Rotmarkpic = Mat::ones(imgInput.rows, imgInput.cols, CV_8UC3);
		Mat Rotnew = Mat::ones(imgInput.rows, imgInput.cols, CV_8UC3);
		Mat thresRot;
		vector<vector<Point>>  contH, contRot;
		vector<Vec4i> hierH, hierRot;

		cv::circle(Rotnew, min_rect.center,6,Scalar(180, 180, 180),FILLED,LINE_AA);

		Rotmarkpic = RotatecorrectImg(-1 * imageParm.correctTheta, Rotnew);
		imgOut = RotatecorrectImg(-1 * imageParm.correctTheta, imgOut);
		cv::inRange(Rotmarkpic, Scalar(175, 175, 175), Scalar(185, 185, 185), thresRot);
		cv::findContours(thresRot, contRot, hierRot, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
		Moments Mans = (moments(contRot[0], false));
		creteriaPoint = (Point2f((Mans.m10 / Mans.m00)+ IMGoffset.x, (Mans.m01 / Mans.m00)+ IMGoffset.y));
	}
	else
	{
		creteriaPoint = Point2f(min_rect.center.x + IMGoffset.x, min_rect.center.y + IMGoffset.y);
	}


	//---衍伸方案 多組Pair 搜尋
#pragma region 先實作


	//int xx=abs(vRegions_Filtered[0].Center().x - vRegions_Filtered[1].Center().x);
	//int yy=abs(vRegions_Filtered[0].Center().y - vRegions_Filtered[1].Center().y);

	//	//刪除頭兩個元素
	//vRegions_Filtered.erase(vRegions_Filtered.begin());
	//vRegions_Filtered.erase(vRegions_Filtered.begin());

	////--判斷是 X 方向配一組 或者 Y方向配一組
	//vector<vector<Point>> vPtC_Rect_Possible;


	//if (xx>yy)
	//{
	//	std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](BlobInfo& a, BlobInfo& b)
	//		{

	//			if (abs(a.Center().y - b.Center().y) < target.TDheight)
	//			{
	//				//---在同一行

	//				if (a.Center().x < b.Center().x)
	//					return true;
	//				else
	//					return false;
	//			}
	//			else if (a.Center().y <b.Center().y)
	//				return true;
	//			else
	//				return false;
	//		});
	//}
	//else
	//{
	//	std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](BlobInfo& a, BlobInfo& b)
	//		{

	//			if (abs(a.Center().x - b.Center().x) < target.TDwidth)
	//			{
	//				//---在同一行

	//				if (a.Center().y < b.Center().y)
	//					return true;
	//				else
	//					return false;
	//			}
	//			else if (a.Center().x < b.Center().x)
	//				return true;
	//			else
	//				return false;
	//		});
	//}




#pragma endregion

	flag = 9;
	Debug.release();
}