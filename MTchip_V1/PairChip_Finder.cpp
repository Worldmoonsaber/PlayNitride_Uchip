#include "MTchip_lib_V1.h"
#include "OpenCV_Extension_Tool.h"


Rect createRectFromBlob(BlobInfo a, BlobInfo b)
{
	Rect rect;

	float Xmax, Ymax, Xmin, Ymin;

	if (a.Xmax() > b.Xmax())
		Xmax = a.Xmax();
	else
		Xmax = b.Xmax();

	if (a.Xmin() > b.Xmin())
		Xmin = b.Xmin();
	else
		Xmin = a.Xmin();

	if (a.Ymax() > b.Ymax())
		Ymax = a.Ymax();
	else
		Ymax = b.Ymax();

	if (a.Ymin() > b.Ymin())
		Ymin = b.Ymin();
	else
		Ymin = a.Ymin();

	return rect = Rect(Xmin, Ymin, (Xmax - Xmin), (Ymax - Ymin));
}

void FindOtherPairs(vector<BlobInfo>& vRegions_Filtered, RotatedRect min_rect,Point ptC,sizeTD_ target,Mat& imgOut,vector<Point2f>& vPtChipPair)
{
	int xx = abs(vRegions_Filtered[0].Center().x - vRegions_Filtered[1].Center().x);
	int yy = abs(vRegions_Filtered[0].Center().y - vRegions_Filtered[1].Center().y);

	float distToCentre = 0.5 * (norm(vRegions_Filtered[0].Center() - min_rect.center) + norm(vRegions_Filtered[1].Center() - min_rect.center)) * 1.5;

	//	//刪除頭兩個元素
	vRegions_Filtered.erase(vRegions_Filtered.begin());
	vRegions_Filtered.erase(vRegions_Filtered.begin());

	////--判斷是 X 方向配一組 或者 Y方向配一組
	//vector<vector<Point>> vPtC_Rect_Possible;
	float x_Interval_Pitch, y_Interval_Pitch;

	if (xx > yy)
	{
		std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](BlobInfo& a, BlobInfo& b)
			{

				if (abs(a.Center().y - b.Center().y) < target.TDheight)
				{
					//---在同一行

					if (a.Center().x < b.Center().x)
						return true;
					else
						return false;
				}
				else if (a.Center().y < b.Center().y)
					return true;
				else
					return false;
			});

		x_Interval_Pitch = 2 * xx;

		for (int i = 0; i < vRegions_Filtered.size() - 1; i++)
			if (abs(vRegions_Filtered[i + 1].Center().y - vRegions_Filtered[i].Center().y) > target.TDheight)
			{
				y_Interval_Pitch = abs(vRegions_Filtered[i + 1].Center().y - vRegions_Filtered[i].Center().y);
				break;
			}

	}
	else
	{
		std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, ptC](BlobInfo& a, BlobInfo& b)
			{

				if (abs(a.Center().x - b.Center().x) < target.TDwidth)
				{
					//---在同一行

					if (a.Center().y < b.Center().y)
						return true;
					else
						return false;
				}
				else if (a.Center().x < b.Center().x)
					return true;
				else
					return false;
			});

		y_Interval_Pitch = 2 * yy;

		for (int i = 0; i < vRegions_Filtered.size() - 1; i++)
			if (abs(vRegions_Filtered[i + 1].Center().x - vRegions_Filtered[i].Center().x) > target.TDwidth)
			{
				x_Interval_Pitch = abs(vRegions_Filtered[i + 1].Center().x - vRegions_Filtered[i].Center().x);
				break;
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

			std::sort(vRegions_Filtered.begin(), vRegions_Filtered.end(), [&, pt](BlobInfo& a, BlobInfo& b)
				{
					return norm(a.Center() - pt) < norm(b.Center() - pt);
				});


			float dist1 = norm(vRegions_Filtered[0].Center() - pt);
			float dist2 = norm(vRegions_Filtered[1].Center() - pt);

			if (dist1 < distToCentre && dist2 < distToCentre)
			{
				Point2f pt2 = 0.5 * (vRegions_Filtered[0].Center() + vRegions_Filtered[1].Center());

				cv::circle(imgOut, pt2, 6, Scalar(255, 100, 255), FILLED, LINE_AA);

				Rect rectChip = createRectFromBlob(vRegions_Filtered[0], vRegions_Filtered[1]);
				rectangle(imgOut, rectChip, Scalar(100, 255, 100), 3);

				vPtChipPair.push_back(pt2);

				vRegions_Filtered.erase(vRegions_Filtered.begin());
				vRegions_Filtered.erase(vRegions_Filtered.begin());
			}


		}

}

void PairChip_Finder(int& flag, Mat imgInput,Mat& imgThres,Mat& imgOut, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point& creteriaPoint, Point IMGoffset, ImgP_ imageParm, vector<Point>& otherCenters)
{
	flag = 0;
	imgOut = imgInput.clone();
	funcThreshold(imgInput, imgThres, thresParm,imageParm,target);
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
			return norm(a.Center() - ptC) < norm(b.Center() - ptC);
		});

	vector<vector<Point>> vPtC_Chip;
	vector<Point> vPtC_Rect;

	for (int i = 0; i < 2; i++)
		vPtC_Chip.push_back(vRegions_Filtered[i].contour());

	drawContours(Debug, vPtC_Chip, -1, Scalar(255, 255, 255));
	findNonZero(Debug, vPtC_Rect);
	cv::RotatedRect min_rect = minAreaRect(vPtC_Rect);
	Point2f vertices2f[4];
	min_rect.points(vertices2f);

	for (int i = 0; i < 4; i++)
		line(imgOut, vertices2f[i], vertices2f[(i + 1) % 4], Scalar(0, 255, 0), 3);


	//-----計算是否偏離

	if (chipsetting.xpitch[0] < norm(min_rect.center - ptC))
	{
		flag = 6; //畫面中心無晶片
		throw "something wrong::potential object doesn't fit suitable dimension";
	}

	//---衍伸方案 多組Pair 搜尋
#pragma region 先實作 輸出多組 Pair
	vector<Point2f> vPtChipPair;

	if (flag == 0 & imageParm.Outputmode>1)
		FindOtherPairs(vRegions_Filtered, min_rect, ptC, target, imgOut, vPtChipPair);
#pragma endregion

	Point ptCarr= min_rect.center+ Point2f(chipsetting.carx, chipsetting.cary);

	std::sort(vPtChipPair.begin(), vPtChipPair.end(), [ptC](Point a, Point b)
		{
			Point2f af = Point2f(a.x, a.y);
			Point2f bf = Point2f(b.x, b.y);
			return norm(af - ptC) < norm(bf - ptC);
		});

	for(int i=0;i< vPtChipPair.size();i++)
		vPtChipPair[i]= vPtChipPair[i]+ Point2f(chipsetting.carx, chipsetting.cary);
	
	//影像中心
	cv::circle(imgOut, ptC + Point2f(chipsetting.carx, chipsetting.cary), 9, Scalar(0, 255, 255), FILLED, LINE_AA);

	// Chip Pair 中心
	cv::circle(imgOut, min_rect.center, 6, Scalar(255, 0, 255), FILLED, LINE_AA);
	cv::line(imgOut, Point(0, ptCarr.y), Point(imgOut.size[1], ptCarr.y), Scalar(255, 255, 255), 1, 8);
	cv::line(imgOut, Point(ptCarr.x, 0), Point(ptCarr.x, imgOut.size[0]), Scalar(255, 255, 255), 1, 8);





	if (imageParm.correctTheta != 0) //平台不能轉的case 
	{
		vector<Point> vPt;
		vPt.push_back(ptCarr);

		vector<Point> vPtOut;
		funcRotatePoint(vPt, vPtOut, imgOut, imageParm.correctTheta,IMGoffset);

		if (vPtOut.size()>0)
			creteriaPoint = vPtOut[0];

		vPt.clear();

		for (int i = 0; i < vPtChipPair.size(); i++)
			vPt.push_back((Point)vPtChipPair[i]);

		funcRotatePoint(vPt, otherCenters, imgOut, imageParm.correctTheta, IMGoffset);
	}
	else
	{
		creteriaPoint = Point(ptCarr.x + IMGoffset.x, ptCarr.y + IMGoffset.y);

		for (int i = 0; i < vPtChipPair.size(); i++)
			otherCenters.push_back(Point(vPtChipPair[i].x + IMGoffset.x, vPtChipPair[i].y + IMGoffset.y));
	}

	flag = 9;
	Debug.release();
}