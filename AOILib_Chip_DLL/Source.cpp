
#include "pch.h"
#include "AOILib_MTUchip_V1.h"


#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp> //mophorlogical operation
#include<opencv2/core.hpp>
#include "../MTchip_V1/MTchip_lib_V1.h"


#pragma region Main
void MTUchip_calcenter(thresP thresParm, ImgP imageParm, SettingP chipsetting, sizeTD target, unsigned int* imageIN,
					unsigned int* imageOUT, unsigned char* imageGray, float boolResult[], float outputLEDX[], float outputLEDY[])
{
	
	Mat rawimg, cropedRImg, gauBGR;
	Mat Gimg, drawF2;

	Point piccenter;
	Point2f creteriaPoint;
	Point IMGoffset=Point(0,0);

	//output parameters::
	Point crossCenter;
	int boolflag = 0;

	Mat image_input(imageParm.rows, imageParm.cols, CV_8UC4, &imageIN[0]); // THIS IS THE INPUT IMAGE, POINTER TO DATA			
	image_input.copyTo(rawimg);

	Mat image_output(imageParm.rows, imageParm.cols, CV_8UC4, &imageOUT[0]);
	Mat thres_output(imageParm.rows, imageParm.cols, CV_8UC1, &imageGray[0]);

	try
	{
		if (rawimg.empty())
		{
			boolflag = 8;
			throw "something wrong::input image failure";
		} //check if image is empty

	} //try loop
	catch (const char* message)
	{

		std::cout << "check catch state:: " << boolflag << endl;


	}//catch loop

	

	if (boolflag == 0) //&& imageParm.Outputmode == 0
	{


		thresP_ _thresParm;


		_thresParm.thresmode = thresParm.thresmode;

		for (int i = 0; i < 3; i++)
		{
			_thresParm.bgmax[i] = thresParm.bgmax[i];
			_thresParm.bgmin[i] = thresParm.bgmin[i];
			_thresParm.fgmax[i] = thresParm.fgmax[i];
			_thresParm.fgmin[i] = thresParm.fgmin[i];
		}

		ImgP_ _imageParm;

		_imageParm.correctTheta = imageParm.correctTheta;
		_imageParm.imgcols = imageParm.cols;
		_imageParm.imgrows = imageParm.rows;
		_imageParm.Outputmode = imageParm.Outputmode;
		_imageParm.PICmode = imageParm.PICmode;

		SettingP_ _chipsetting;

		_chipsetting.carx = chipsetting.carx;
		_chipsetting.cary = chipsetting.cary;

		for (int i = 0; i < 4; i++)
			_chipsetting.interval[i] = chipsetting.interval[i];

		for (int i = 0; i < 3; i++)
		{
			_chipsetting.xpitch[i] = chipsetting.xpitch[i];
			_chipsetting.ypitch[i] = chipsetting.ypitch[i];
		}


		sizeTD_ _target;

		_target.TDheight = target.TDheight;
		_target.TDmaxH = target.TDmaxH;
		_target.TDmaxW = target.TDmaxW;
		_target.TDminH = target.TDminH;
		_target.TDminW = target.TDminW;
		_target.TDwidth = target.TDwidth;


		rawimg.copyTo(cropedRImg);

		if (imageParm.correctTheta != 0)
			cropedRImg = RotatecorrectImg(-1*imageParm.correctTheta, cropedRImg);

		creteriaPoint = find_piccenter(cropedRImg);


		if (imageParm.Outputmode == 0)
		{	
			std::tie(boolflag, Gimg, crossCenter, drawF2) = Uchip_singlephaseDownV3(boolflag, cropedRImg, _thresParm, _chipsetting, _target, creteriaPoint, IMGoffset, _imageParm);
		}
		else
		{
			vector<Point> vPt;
			PairChip_Finder(boolflag, cropedRImg, Gimg, drawF2, _thresParm, _chipsetting, _target, crossCenter, IMGoffset, _imageParm,vPt);

			if (vPt.size() > 0)
				for (int i = 1; i < sizeof(outputLEDX) / sizeof(outputLEDX[0]); i++)
				{
					if (i - 1 == vPt.size())
						break;
					outputLEDX[i] = vPt[i - 1].x;
					outputLEDY[i] = vPt[i - 1].y;
				}


		}
		
		
	}

	std::cout << "check img state:: " << boolflag << endl;
	std::cout << "check center is ::" << crossCenter << endl;

	if(!cropedRImg.empty())
		cropedRImg.release();

	/*  :::::::OUTPUT area:::::::  */
	outputLEDX[0] = crossCenter.x ;
	outputLEDY[0] = crossCenter.y ;
	Gimg.copyTo(thres_output);
	drawF2.copyTo(image_output);
	boolResult[0] = boolflag;
}


#pragma endregion Main



