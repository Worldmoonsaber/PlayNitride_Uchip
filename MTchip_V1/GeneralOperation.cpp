

#include "MTchip_lib_V1.h"



Point find_piccenter(Mat src) {
	int piccenterx = int(src.size().width * 0.5);
	int piccentery = int(src.size().height * 0.5);
	Point piccenter = Point(piccenterx, piccentery);
	return piccenter;
}

Mat CropIMG(Mat img, Rect size)
{
	Mat croppedIMG;
	img(size).copyTo(croppedIMG);
	return croppedIMG;

}

Mat CropImgFromChipSetting(Mat img, SettingP_ chipsetting, sizeTD_ target, ImgP_ imageParm, int& flag,Point& PicCenterOut,Point& ImgOffsetOut)
{
	flag = 0;

	int xTolerance= int(imageParm.imgcols * 0.5);
	int yTolerance = int(imageParm.imgrows * 0.5);

	PicCenterOut = Point(chipsetting.carx, chipsetting.cary);

	//---是否超過影像範圍
	if (chipsetting.carx < xTolerance || chipsetting.cary < yTolerance)
		flag = 7;

	ImgOffsetOut.x = PicCenterOut.x - int(imageParm.imgcols * 0.5);
	ImgOffsetOut.y = PicCenterOut.y - int(imageParm.imgrows * 0.5);

	//---是否超過影像範圍
	if (chipsetting.carx > img.cols- xTolerance || chipsetting.cary > img.rows- yTolerance)
		flag = 7;

	if (flag != 7)
	{
		Rect Cregion(ImgOffsetOut.x, ImgOffsetOut.y, imageParm.imgcols, imageParm.imgrows);
		Mat CropImg = CropIMG(img, Cregion);
		return CropImg;
	}
	else
	{
		Rect Cregion(0,0, imageParm.imgcols, imageParm.imgrows);
		Mat CropImg = CropIMG(img, Cregion);
		return CropImg;
	}
}

int findBoundary(Mat creteriaIMG, Rect inirect, char direction)
{
	int step = 1;
	auto findRecr = inirect;
	int BoundaryVal;

	switch (direction)
	{
	case 'L':
		while (true)
		{
			//const auto count = cv::countNonZero(inverted_mask(inside_rect));
			const auto count = cv::countNonZero(creteriaIMG(findRecr));
			if (count > 0)
			{
				BoundaryVal = findRecr.x;
				break;
			}
			findRecr.x -= step;
		}
		break;
	case 'T':
		while (true)
		{
			const auto count = cv::countNonZero(creteriaIMG(findRecr));
			if (count > 0)
			{
				BoundaryVal = findRecr.y;
				break;
			}
			findRecr.y -= step;
		}
		break;
	case 'R':
		while (true)
		{
			const auto count = cv::countNonZero(creteriaIMG(findRecr));
			if (count > 0)
			{
				BoundaryVal = findRecr.x;
				break;
			}
			findRecr.x += step;
		}
		break;

		break;
	case 'B':
		while (true)
		{
			const auto count = cv::countNonZero(creteriaIMG(findRecr));
			if (count > 0)
			{
				BoundaryVal = findRecr.y;
				break;
			}
			findRecr.y += step;
		}
		break;

	default:
		std::cout << "****** Error case mode ******" << endl;
		break;

	}


	std::cout << "finish findboundary~" << endl;
	std::cout << "fi";
	return BoundaryVal;
}

std::tuple<Rect, Point>FindMaxInnerRect(Mat src, Mat colorSRC, sizeTD_ target, Point TDcenter)
{
	//output:::
	Rect innerboundary;
	Point center = TDcenter;
	Mat markcolor = Mat::zeros(colorSRC.size(), CV_8UC4);
	colorSRC.copyTo(markcolor);
	cv::circle(markcolor, center, 2, Scalar(180, 180, 180), 5);
	cv::circle(src, center, 2, Scalar(180, 180, 180), 5);
	//
	//find inner rect:
	Size ksize;
	Mat src2;
	ksize = Size(15, 15);
	Mat Kcomclose = Mat::ones(ksize, CV_8UC1);
	cv::morphologyEx(src, src2, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 4);//4
	threshold(src2, src2, 175, 255, THRESH_BINARY);
	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours, reqCon;
	vector<Point> approx;
	Rect retCOMP;
	vector<Point> reqCenter;

	Rect fineRect;

	cv::findContours(src2, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	double areasrc = cv::contourArea(contours[0]);

	for (int i = 0; i < contours.size(); i++)
	{
		areasrc = cv::contourArea(contours[i]);
		if (areasrc < target.TDwidth * target.TDminH * target.TDheight)
		{
			Rect bdrect = cv::boundingRect(contours[i]);
			cv::rectangle(src2, bdrect, Scalar(255, 255, 255), -1);
		}
	}
	cv::findContours(src2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::drawContours(src2, contours, -1, Scalar(255, 255, 255), -1);
	//cv::approxPolyDP(contours[0], approx, arcLength(contours[0], true) * 0.02, true); //

	//std::cout << "check approx size : " << approx.size() << endl;

	//Step.NEW7-find inner rect (via tiny scanning mechanism) ::
	cv::Mat inverted_mask;
	cv::bitwise_not(src2, inverted_mask);
	cv::Mat pointsmsk = Mat::zeros(src.size(), CV_8UC1);;
	cv::findNonZero(src2, pointsmsk);
	const cv::Rect outside_rect = cv::boundingRect(pointsmsk);
	Mat TDrect = Mat::zeros(src.size(), CV_8UC1);
	src.copyTo(TDrect);
	cv::rectangle(TDrect, outside_rect, Scalar(180, 180, 180), 1); //Scalar(0, 0, 0)

	/*std::cout << "check area : " << outside_rect.width * outside_rect.height << " // " << target.TDwidth * target.TDheight << endl;
	std::cout << "calculate area : " << outside_rect.width * outside_rect.height << " // " << target.TDwidth * target.TDheight << endl;*/
	int step_w, step_h;


	/*if ((outside_rect.width * outside_rect.height) / (target.TDwidth * target.TDheight) > 0.8
		&& (outside_rect.width * outside_rect.height) / (target.TDwidth * target.TDheight) < 1.15 )*/


	std::cout << "[select inner rect...]" << endl;
	if (outside_rect.width > outside_rect.height)
	{
		step_w = 1;//2
		step_h = 1;//1
	}
	else
	{
		step_w = 1;//1
		step_h = 1;//2
	}


	auto inside_rect = outside_rect;


	while (true)
	{
		//const auto count = cv::countNonZero(inverted_mask(inside_rect));
		const auto count = cv::countNonZero(inverted_mask(inside_rect));


		if (count == 0)
		{
			// we found a rectangle we can use!
			break;
		}

		inside_rect.x += step_w;
		inside_rect.y += step_h;
		inside_rect.width -= (step_w * 2);
		inside_rect.height -= (step_h * 2);
	}



	//cv::rectangle(TDrect, inside_rect, Scalar(100, 100, 100), 1); //Scalar(0, 0, 0)

	/*std::cout << "check inside rect:: " << inside_rect << endl;
	std::cout << "check outside_rect:: " << outside_rect << endl;*/

	//Step.NEW8-find inner rect boundary ::
	
	cv::rectangle(inverted_mask, Rect(0, 0, inverted_mask.size().width, inverted_mask.size().height), Scalar(255, 255, 255), 1);
	Rect line = Rect(inside_rect.x, inside_rect.y, inside_rect.width, 1);
	//cv::rectangle(gamimg, line, Scalar(0, 0, 0), 1); //Scalar(0, 0, 0)
	const auto count = cv::countNonZero(inverted_mask(line));
	//std::cout << "999999999999999999999999999999999: " << count << endl;
	int leftBound;
	//Rect Leftline = Rect(inside_rect.x, inside_rect.y, 1, inside_rect.height); //360,355 
	Rect Leftline = Rect(int(inside_rect.x+1), inside_rect.y + int(inside_rect.height * 0.15), 1, int(0.7 * inside_rect.height)); //360,355 
	//cv::rectangle(colorSRC, Leftline, Scalar(88, 50, 155), 2);
	leftBound = findBoundary(inverted_mask, Leftline, 'L');
	//std::cout << "check left boundary " << leftBound << endl;


	int topBound;
	//Rect Topline = Rect(inside_rect.x, inside_rect.y, inside_rect.width, 1);
	Rect Topline = Rect(inside_rect.x + int(0.15 * inside_rect.width), int(inside_rect.y+1), int(0.7 * inside_rect.width), 1);
	topBound = findBoundary(inverted_mask, Topline, 'T');
	//std::cout << "check Top boundary " << topBound << endl;

	int RightBound;
	//Rect Rightline = Rect(inside_rect.x + inside_rect.width, inside_rect.y, 1, inside_rect.height);
	Rect Rightline = Rect(inside_rect.x + int(inside_rect.width-1), inside_rect.y + int(inside_rect.height * 0.15), 1, int(0.7 * inside_rect.height));
	//cv::rectangle(colorSRC, Rightline, Scalar(88, 50, 155), 2);
	RightBound = findBoundary(inverted_mask, Rightline, 'R');
	//std::cout << "check right boundary " << RightBound << endl;

	int BottomBound;
	//Rect bottomline = Rect(inside_rect.x, inside_rect.y + inside_rect.height, inside_rect.width, 1);
	Rect bottomline = Rect(inside_rect.x + int(0.15 * inside_rect.width), inside_rect.y +int( inside_rect.height-1), int(0.7 * inside_rect.width), 1);
	BottomBound = findBoundary(inverted_mask, bottomline, 'B');
	//std::cout << "check bottom boundary " << BottomBound << endl;

	//innerboundary = Rect(leftBound, topBound, (RightBound - leftBound), (BottomBound - topBound));

	innerboundary = Rect(leftBound, topBound, (RightBound - leftBound), (BottomBound - topBound));

	center = Point(int(innerboundary.x + innerboundary.width * 0.5), int(innerboundary.y + innerboundary.height * 0.5));
	//std::cout << "previous center is :: " << center << endl;

	cv::rectangle(markcolor, innerboundary, Scalar(0, 255, 255), 1);

	//std::cout << "innerboundary.width " << innerboundary.width << "/*/* " << target.TDwidth << endl;





	//Step.NEW9-Mark inner rect::
	cv::rectangle(markcolor, innerboundary, Scalar(0, 0, 255), 1);
	cv::rectangle(markcolor, fineRect, Scalar(255, 0, 0), 1);
	cv::rectangle(TDrect, innerboundary, Scalar(50, 50, 50), 2);
	cv::circle(markcolor, center, 2, Scalar(20, 20, 20), 5);
	//check area::
	/*std::cout << "check dimension-width::: " << innerboundary.width << " ||| " << target.TDwidth << endl;
	std::cout << "check dimension-height::: " << innerboundary.height << " ||| " << target.TDheight << endl;
	std::cout << "check center::: " << center << endl;*/

	std::cout << "fini" << endl;

	//
	return { innerboundary,center };

}

Mat RotatecorrectImg(double Rtheta, Mat src)
{
	Point2f center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
	// using getRotationMatrix2D() to get the rotation matrix
	Mat rotation_matix = getRotationMatrix2D(center, Rtheta, 1.0);
	//angle:: ++ mean counter-clockwise rotation (the coordinate origin is assumed to be the top-left corner)

	// we will save the resulting image in rotated_image matrix
	Mat rotated_image;
	// rotate the image using warpAffine
	warpAffine(src, rotated_image, rotation_matix, src.size());
	Mat patchrotaIMG;
	rotated_image.copyTo(patchrotaIMG);

	for (int y = 0; y < rotated_image.rows; y++)
	{
		for (int x = 0; x < rotated_image.cols; x++)
		{
			if (rotated_image.at<Vec3b>(Point(x, y)) == Vec3b(0, 0, 0))
			{
				patchrotaIMG.at<Vec3b>(Point(x, y)) = Vec3b(255, 255, 255);
			}
		}
	}

	return patchrotaIMG;
}

void funcThreshold(Mat ImgInput, Mat& ImgThres, thresP_ thresParm, ImgP_ imageParm, sizeTD_ target)
{
	ImgThres = Mat::zeros(ImgInput.rows, ImgInput.cols, CV_8UC1);
	//Thres parameters:::
	Mat Gimg, gauGimh;
	Mat gauBGR;

	Mat adptThres;
	Mat HIGHthres;
	int adaptWsize = 3;
	int adaptKsize = 2;

	//Step.1  pre-processing to enhance contrast
	if (thresParm.thresmode == 0
		|| thresParm.thresmode == 3
		|| thresParm.thresmode == 4)
	{
		ImgInput.convertTo(ImgInput, -1, 1.2, 0);

		cv::GaussianBlur(ImgInput, gauBGR, Size(0, 0), 13);
		cv::addWeighted(ImgInput, 1.5, gauBGR, -0.7, 0.0, ImgInput);

		//Step.2  pre-processing of denoise
		cv::cvtColor(ImgInput, Gimg, COLOR_RGB2GRAY);
		cv::fastNlMeansDenoising(Gimg, gauGimh, 3, 7, 21);
	}
	else
	{
		cv::cvtColor(ImgInput, Gimg, COLOR_RGB2GRAY);
	}


	//Step.3 threshold filtering
	if (thresParm.thresmode == 0)
	{
		Scalar maxthres = Scalar(thresParm.fgmax[imageParm.PICmode], thresParm.fgmax[imageParm.PICmode], thresParm.fgmax[imageParm.PICmode]);
		Scalar minthres = Scalar(thresParm.fgmin[imageParm.PICmode], thresParm.fgmin[imageParm.PICmode], thresParm.fgmin[imageParm.PICmode]);
		cv::inRange(gauGimh, minthres, maxthres, HIGHthres);
		cv::medianBlur(HIGHthres, ImgThres, 17);

		Size S_kermask, S_kernelLOW2, S_Kcomclose;

		if (target.TDheight < target.TDwidth)
		{
			S_kermask = Size(10, 1);
			S_kernelLOW2 = Size(1, 7);
			S_Kcomclose = Size(10, 5);
		}
		else
		{
			S_kermask = Size(1, 10);
			S_kernelLOW2 = Size(7, 1);
			S_Kcomclose = Size(5, 10);
		}

		Mat Kcomclose = Mat::ones(S_Kcomclose, CV_8UC1);  //Size(10,5)
		cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 1);//1 //2


		if (thresParm.bgmin[imageParm.PICmode] != 99999 && thresParm.bgmax[imageParm.PICmode] != 99999)
		{
			Scalar maxthres = Scalar(thresParm.bgmax[imageParm.PICmode], thresParm.bgmax[imageParm.PICmode], thresParm.bgmax[imageParm.PICmode]);
			Scalar minthres = Scalar(thresParm.bgmin[imageParm.PICmode], thresParm.bgmin[imageParm.PICmode], thresParm.bgmin[imageParm.PICmode]);
			cv::inRange(gauGimh, minthres, maxthres, HIGHthres);
			cv::medianBlur(HIGHthres, HIGHthres, 17);
			ImgThres = ImgThres + HIGHthres;
			cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 1);//1 //2
		}



	}
	else if(thresParm.thresmode==3 || thresParm.thresmode == 4)
	{
		int nThresholdType = THRESH_BINARY_INV;

		if (thresParm.thresmode == 4)
			nThresholdType = THRESH_BINARY;

		if (thresParm.bgmax[imageParm.PICmode] & 1)
		{
			adaptWsize = thresParm.bgmax[imageParm.PICmode];
			adaptKsize = thresParm.fgmax[imageParm.PICmode];
		}
		else
		{
			adaptWsize = thresParm.bgmax[imageParm.PICmode] + 1;
			adaptKsize = thresParm.fgmax[imageParm.PICmode];
		}
		adaptiveThreshold(gauGimh, adptThres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, nThresholdType, adaptWsize, adaptKsize);//55,1 //ADAPTIVE_THRESH_MEAN_C

		cv::medianBlur(adptThres, ImgThres, 7);
		Mat Kcomclose = Mat::ones(Size(5, 5), CV_8UC1);  //Size(10,5)
		cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_CLOSE, Kcomclose, Point(-1, -1), 1);//1 //2
		Kcomclose.release();
	}
	else // thresParm.thresmode==5 以及防呆
	{
		Mat ImgThres2;
		adaptiveThreshold(Gimg, ImgThres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 91, 0);//55,1 //ADAPTIVE_THRESH_MEAN_C
		adaptiveThreshold(Gimg, ImgThres2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 91, 0);//55,1 //ADAPTIVE_THRESH_MEAN_C

		//cv::medianBlur(adptThres, ImgThres, 7);
		Mat Kcomopen = Mat::ones(Size(10, 10), CV_8UC1);  //Size(10,5)
		Mat Kcomclose = Mat::ones(Size(5, 5), CV_8UC1);  //Size(10,5)

		cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_OPEN, Kcomopen, Point(-1, -1), 1);//1 //2
		cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_CLOSE, Kcomopen, Point(-1, -1), 1);//1 //2

		cv::morphologyEx(ImgThres2, ImgThres2, cv::MORPH_OPEN, Kcomopen, Point(-1, -1), 1);//1 //2
		cv::morphologyEx(ImgThres2, ImgThres2, cv::MORPH_CLOSE, Kcomopen, Point(-1, -1), 1);//1 //2

		bitwise_or(ImgThres, ImgThres2, ImgThres);
		Mat Kcomclose2 = Mat::ones(Size(3, 3), CV_8UC1);  //Size(10,5)
		cv::morphologyEx(ImgThres, ImgThres, cv::MORPH_CLOSE, Kcomclose2, Point(-1, -1), 1);//1 //2

		ImgThres2.release();
		Kcomclose.release();
		Kcomopen.release();
		Kcomclose2.release();
	}

	gauBGR.release();
	gauGimh.release();
	Gimg.release();
	HIGHthres.release();
}

void funcRotatePoint(vector<Point> vPt, vector<Point>& vPtOut,Mat& marksize,float correctTheta, Point IMGoffset)
{
	vector<vector<Point>>  contH, contRot;
	vector<Vec4i> hierH, hierRot;
	Mat thresRot;

	Mat Rotmarkpic = Mat::ones(marksize.rows, marksize.cols, CV_8UC3);
	Mat Rotnew = Mat::ones(marksize.rows, marksize.cols, CV_8UC3);

	for (int i = 0; i < vPt.size(); i++)
	{
		cv::circle(Rotnew,
			(vPt[i]), //coordinate
			6, //radius
			Scalar(180, 180, 180),  //color
			FILLED,
			LINE_AA);

	}

	Rotmarkpic = RotatecorrectImg(-1 * correctTheta, Rotnew);
	marksize = RotatecorrectImg(-1 * correctTheta, marksize);
	cv::inRange(Rotmarkpic, Scalar(175, 175, 175), Scalar(185, 185, 185), thresRot);
	cv::findContours(thresRot, contRot, hierRot, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	for (int i = 0; i < contRot.size(); i++)
	{
		Moments Mans = (moments(contRot[0], false));
		vPtOut.push_back(Point2i((Point2f((Mans.m10 / Mans.m00), (Mans.m01 / Mans.m00)))) + IMGoffset);
	}

	Rotmarkpic.release();
	Rotnew.release();
}