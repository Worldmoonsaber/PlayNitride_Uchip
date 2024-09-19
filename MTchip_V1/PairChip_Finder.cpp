#include "MTchip_lib_V1.h"



Rect createRectFromContour(vector<cv::Point> a, vector<cv::Point> b)
{
	Rect rect;

	int Xmax=INT_MIN, Ymax = INT_MIN, Xmin = INT_MAX, Ymin = INT_MAX;


	for (int i = 0; i < a.size(); i++)
	{
		if (Xmax < a[i].x)
			Xmax = a[i].x;

		if (Ymax < a[i].y)
			Ymax = a[i].y;

		if (Xmin > a[i].x)
			Xmin = a[i].x;

		if (Ymin > a[i].y)
			Ymin = a[i].y;
	}


	for (int i = 0; i < b.size(); i++)
	{
		if (Xmax < b[i].x)
			Xmax = b[i].x;

		if (Ymax < b[i].y)
			Ymax = b[i].y;

		if (Xmin > b[i].x)
			Xmin = b[i].x;

		if (Ymin > b[i].y)
			Ymin = b[i].y;
	}



	return rect = Rect(Xmin, Ymin, (Xmax - Xmin), (Ymax - Ymin));
}

void FindOtherPairs(vector<tuple<Point2f, vector<cv::Point>>>& vRegions_Filtered, RotatedRect min_rect,Point ptC,sizeTD_ target,Mat& imgOut,vector<Point2f>& vPtChipPair)
{
	Point2f pt1= std::get<0>(vRegions_Filtered[0]);
	Point2f pt2= std::get<0>(vRegions_Filtered[1]);
	;

	int xx = abs(pt1.x - pt2.x);
	int yy = abs(pt1.y - pt2.y);

	float distToCentre = 0.5 * (norm(pt1 - min_rect.center) + norm(pt2 - min_rect.center)) * 1.5;

	//	//刪除頭兩個元素
	vRegions_Filtered.erase(vRegions_Filtered.begin());
	vRegions_Filtered.erase(vRegions_Filtered.begin());

	////--判斷是 X 方向配一組 或者 Y方向配一組
	//vector<vector<Point>> vPtC_Rect_Possible;
	float x_Interval_Pitch, y_Interval_Pitch;

	if (xx > yy)
	{
		std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](tuple<Point2f, vector<cv::Point>>& a, tuple<Point2f, vector<cv::Point>>& b)
			{
				Point2f a1 = std::get<0>(a);
				Point2f b1 = std::get<0>(b);

				if (abs(a1.y - b1.y) < target.TDheight)
				{
					//---在同一行

					if (a1.x < b1.x)
						return true;
					else
						return false;
				}
				else if (a1.y < b1.y)
					return true;
				else
					return false;
			});

		x_Interval_Pitch = 2 * xx;

		for (int i = 0; i < vRegions_Filtered.size() - 1; i++)
		{

			Point2f pt_iand1 = std::get<0>(vRegions_Filtered[i+1]);
			Point2f pt_i = std::get<0>(vRegions_Filtered[i]);

			if (abs(pt_iand1.y - pt_i.y) > target.TDheight)
			{
				y_Interval_Pitch = abs(pt_iand1.y - pt_i.y);
				break;
			}
		}

	}
	else
	{
		std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](tuple<Point2f, vector<cv::Point>>& a, tuple<Point2f, vector<cv::Point>>& b)
			{
				Point2f a1 = std::get<0>(a);
				Point2f b1 = std::get<0>(b);


				if (abs(a1.x - b1.x) < target.TDwidth)
				{
					//---在同一行

					if (a1.y < b1.y)
						return true;
					else
						return false;
				}
				else if (a1.x < b1.x)
					return true;
				else
					return false;
			});

		y_Interval_Pitch = 2 * yy;

		for (int i = 0; i < vRegions_Filtered.size() - 1; i++)
		{
			Point2f pt_iand1 = std::get<0>(vRegions_Filtered[i + 1]);
			Point2f pt_i = std::get<0>(vRegions_Filtered[i]);

			if (abs(pt_iand1.x - pt_i.x) > target.TDwidth)
			{
				x_Interval_Pitch = abs(pt_iand1.x - pt_i.x);
				break;
			}
		}
	}

	vector<float> vX; vX.clear();
	vX.push_back(min_rect.center.x);

	float vXe = min_rect.center.x;

	while (true)
	{
		vXe -= x_Interval_Pitch;

		if (vXe < 0)
			break;
		else
			vX.push_back(vXe);
	}

	vXe = min_rect.center.x;

	while (true)
	{
		vXe += x_Interval_Pitch;

		if (vXe > imgOut.cols)
			break;
		else
			vX.push_back(vXe);
	}

	for (int i = 0; i < vX.size(); i++)
		cv::line(imgOut, Point(vX[i], 0), Point(vX[i], imgOut.size[0]), Scalar(255, 255, 255), 1, 8);

	vector<float> vY;
	vY.push_back(min_rect.center.y);

	float vYe = min_rect.center.y;

	while (true)
	{
		vYe -= y_Interval_Pitch;

		if (vYe < 0)
			break;
		else
			vY.push_back(vYe);
	}

	vYe = min_rect.center.y;

	while (true)
	{
		vYe += y_Interval_Pitch;

		if (vYe > imgOut.rows)
			break;
		else
			vY.push_back(vYe);
	}

	for (int i = 0; i < vY.size(); i++)
		cv::line(imgOut, Point(0, vY[i]), Point(imgOut.size[1], vY[i]), Scalar(255, 255, 255), 1, 8);

	for (int i = 0; i < vX.size(); i++)
		for (int j = 0; j < vY.size(); j++)
		{
			if (i == 0 & j == 0)
				continue;

			if (vRegions_Filtered.size() < 2)
			{
				break;
			}

			Point2f pt = Point(vX[i], vY[j]);

			std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, pt](tuple<Point2f, vector<cv::Point>>& a, tuple<Point2f, vector<cv::Point>>& b)
				{
					Point2f a1 = std::get<0>(a);
					Point2f b1 = std::get<0>(b);

					return norm(a1 - pt) < norm(b1 - pt);
				});


			float dist1 = norm(std::get<0>(vRegions_Filtered[0]) - pt);
			float dist2 = norm(std::get<0>(vRegions_Filtered[1]) - pt);

			if (dist1 < distToCentre && dist2 < distToCentre)
			{
				Point2f pt2 = 0.5 * (std::get<0>(vRegions_Filtered[0]) + std::get<0>(vRegions_Filtered[1]));

				cv::circle(imgOut, pt2, 6, Scalar(255, 100, 255), FILLED, LINE_AA);

				Rect rectChip = createRectFromContour(std::get<1>(vRegions_Filtered[0]), std::get<1>(vRegions_Filtered[1]));
				rectangle(imgOut, rectChip, Scalar(100, 255, 100), 3);

				vPtChipPair.push_back(pt2);

				vRegions_Filtered.erase(vRegions_Filtered.begin());
				vRegions_Filtered.erase(vRegions_Filtered.begin());
			}


		}

}

void PairChip_Finder(int& flag, Mat imgInput,Mat& imgThres,Mat& imgOut, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, cv::Point& creteriaPoint, cv::Point IMGoffset, ImgP_ imageParm, vector<Point>& otherCenters)
{
	flag = 0;
	imgOut = imgInput.clone();
	funcThreshold(imgInput, imgThres, thresParm,imageParm,target);
	int maxArea= target.TDheight*target.TDwidth*target.TDmaxW* target.TDmaxH;
	int minArea= target.TDheight * target.TDwidth * target.TDminW* target.TDminH;

	vector<vector<cv::Point>>  contH, contRot;
	vector<Vec4i> hierH, hierRot;

	//findContours(imgThres, contH, hierH
	//	RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
	findContours(imgThres, contH, hierH, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


	if (contH.size() == 0)
	{
		flag = 1;
		throw "something wrong::threshold value issue";
	}

	vector<tuple<Point2f, vector<cv::Point>>> vContour_Filtered;
	//--依照條件過濾

	for (int i = 0; i < contH.size(); i++)
	{
		Rect retCOMP = cv::boundingRect(contH[i]);
		float areacomthres = cv::contourArea(contH[i]);

		if (retCOMP.area() < target.TDwidth * target.TDminW * target.TDheight * target.TDminH)
			continue;

		float rectangularity = areacomthres / retCOMP.area();

		if (rectangularity < 0.5)
			continue;

		if (retCOMP.width > target.TDwidth * target.TDminW
			&& retCOMP.height > target.TDheight * target.TDminH
			&& retCOMP.width < target.TDwidth * target.TDmaxW
			&& retCOMP.height < target.TDheight * target.TDmaxH
			)
		{
			Moments M = (moments(contH[i], false));
			tuple<Point2f, vector<cv::Point>> obj(Point2f((M.m10 / M.m00), (M.m01 / M.m00)), contH[i]);
			rectangle(imgThres, retCOMP, cv::Scalar(255, 255, 255), cv::FILLED);
			rectangle(imgOut, retCOMP, cv::Scalar(255, 255, 255), 5);

			vContour_Filtered.push_back(obj);
		}

	}

	if (vContour_Filtered.size() <2 )
	{
		flag = 2;
		throw "something wrong::potential object doesn't fit suitable dimension";
	}

	//----DebugImg
	Mat Debug = Mat::zeros(imgInput.rows, imgInput.cols, CV_8UC1);
	Point2f ptC = find_piccenter(imgInput);

	std::sort(vContour_Filtered.begin(), vContour_Filtered.end(), [&,ptC](tuple<Point2f, vector<cv::Point>>& a,tuple<Point2f, vector<cv::Point>>& b)
		{
			Point2f a1=std::get<0>(a);
			Point2f b1 = std::get<0>(b);

			return norm(a1 - ptC) < norm(b1 - ptC);
		});

	vector<vector<cv::Point>> vPtC_Chip;
	vector<cv::Point>vPtC_Rect;

	for (int i = 0; i < 2; i++)
		vPtC_Chip.push_back(std::get<1>(vContour_Filtered[i]));

	cv::drawContours(Debug, vPtC_Chip, -1, cv::Scalar(255, 255, 255));
	cv::findNonZero(Debug, vPtC_Rect);
	cv::RotatedRect min_rect = minAreaRect(vPtC_Rect);
	Point2f vertices2f[4];
	min_rect.points(vertices2f);

	for (int i = 0; i < 4; i++)
		line(imgOut, vertices2f[i], vertices2f[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);


	//-----計算是否偏離

	if (chipsetting.xpitch[0] < norm(min_rect.center - ptC))
	{
		flag = 6; //畫面中心無晶片
		throw "something wrong::potential object doesn't fit suitable dimension";
	}

	//vector<Point2f> vPtChipPair;


	//---衍伸方案 多組Pair 搜尋
#pragma region 先實作 輸出多組 Pair
	vector<Point2f> vPtChipPair;

	if (flag == 0 & imageParm.Outputmode>1)
		FindOtherPairs(vContour_Filtered, min_rect, ptC, target, imgOut, vPtChipPair);
#pragma endregion

	cv::Point ptCarr = min_rect.center;// +Point2f(chipsetting.carx, chipsetting.cary);

	std::sort(vPtChipPair.begin(), vPtChipPair.end(), [ptC](cv::Point a, cv::Point b)
		{
			Point2f af = Point2f(a.x, a.y);
			Point2f bf = Point2f(b.x, b.y);
			return norm(af - ptC) < norm(bf - ptC);
		});

	for (int i = 0; i < vPtChipPair.size(); i++)
		vPtChipPair[i] = vPtChipPair[i];// +Point2f(chipsetting.carx, chipsetting.cary);
	
	//影像中心
	cv::circle(imgOut, ptC , 9, cv::Scalar(0, 255, 255), FILLED, LINE_AA);

	cv::line(imgOut, cv::Point(0, ptCarr.y), cv::Point(imgOut.size[1], ptCarr.y), cv::Scalar(255, 255, 255), 1, 8);
	cv::line(imgOut, cv::Point(ptCarr.x, 0), cv::Point(ptCarr.x, imgOut.size[0]), cv::Scalar(255, 255, 255), 1, 8);
	// Chip Pair 中心
	cv::circle(imgOut, min_rect.center, 6, cv::Scalar(255, 0, 255), FILLED, LINE_AA);


	if (imageParm.correctTheta != 0) //平台不能轉的case 
	{
		vector<cv::Point> vPt;
		vPt.push_back(ptCarr);

		vector<cv::Point> vPtOut;
		funcRotatePoint(vPt, vPtOut, imgOut, imageParm.correctTheta,IMGoffset);

		if (vPtOut.size()>0)
			creteriaPoint = vPtOut[0];

		vPt.clear();

		for (int i = 0; i < vPtChipPair.size(); i++)
			vPt.push_back((cv::Point)vPtChipPair[i]);

		funcRotatePoint(vPt, otherCenters, imgOut, imageParm.correctTheta, IMGoffset);
	}
	else
	{
		creteriaPoint = cv::Point(ptCarr.x + IMGoffset.x, ptCarr.y + IMGoffset.y);

		for (int i = 0; i < vPtChipPair.size(); i++)
			otherCenters.push_back(cv::Point(vPtChipPair[i].x + IMGoffset.x, vPtChipPair[i].y + IMGoffset.y));
	}

	flag = 9;
	Debug.release();
}