
#include "MTchip_lib_V1.h"

int main()
{
	//--- Debug ����
	SettingP_ chipsetting;
	thresP_ thresParm;
	ImgP_ imageParm;
	sizeTD_ target;



	imageParm.imgcols = 1500; //800 ;900-1600
	imageParm.imgrows = 1500;

	imageParm.Outputmode = 0; //0:center coord ; 1: multiple mode
	imageParm.PICmode = 0;  
	chipsetting.interval[0] = 0; 
	chipsetting.xpitch[0] = 500; 
	chipsetting.carx = 3800;
	chipsetting.cary = 2000;

	//positive: counterclockwise   / negative:clockwise
	imageParm.correctTheta = 0; 
	
	/////////////////////////////////////////
	Mat rawimg, cropedRImg;
	int picorder;
	Point piccenter;
	Point IMGoffset;
	Point2f creteriaPoint;


	//output parameters::
	Mat ReqIMG, marksize;
	Point crossCenter;
	int boolflag = 0;//11

	//operating mode
	//C:\Image\Uchip\MT1\240522_MT1Ukey_MT1Uchip1836\240522_MT1Ukey_MT1Uchip1836\chip
	rawimg = imread("C:\\Image\\4X4\\1836\\183607.bmp");

		// Image source input: IMG format:RGB
		//try
		//{
		//	std::tie(picorder, rawimg) = Inputfunction();
		//	if (rawimg.empty())
		//	{
		//		boolflag = 8;
		//		throw "something wrong::input image failure";
		//	} //check if image is empty

		//} //try loop
		//catch (const char* message)
		//{

		//	std::cout << "check catch state:: " << boolflag << endl;

		//}//catch loop

	float outputLEDX[500];
	float outputLEDY[500];

		/////
		//vector<float> sizelist;
		//vector<int>threslist;

		//std::tie(sizelist, threslist) = dict_rectregion(picorder);

		target.TDwidth = 350;
		target.TDmaxW = 1.5;
		target.TDminW = 0.8;

		target.TDheight = 180;
		target.TDmaxH = 1.5;
		target.TDminH = 0.7;

		//thresParm = { 4,{200,107,107},{99999,99999,99999} ,{1,9,9}, {99999,99999,99999} }; //MTchip-G�BB[0]
		//thresParm = { 3,{280,107,107},{99999,99999,99999} ,{9,9,9}, {99999,99999,99999} };//MTchip-R[0]
		//thresParm = { 3,{204,107,107},{99999,99999,99999} ,{3,9,9}, {99999,99999,99999} };//MTchip-R[0]
		//thresParm = { 4,{107,107,107},{99999,99999,99999} ,{9,9,9}, {0,0,0} }; //3:AutoUP ; 4AutoDown 2040
		//thresParm = { 3,{107,107,107},{99999,99999,99999} ,{9,9,9}, {99999,99999,99999} }; //3:AutoUP ; 4AutoUP
		//thresParm = { 0,{99999,99999,99999},{99999,99999,99999} ,{threslist[0],threslist[0],threslist[0]}, {0,0,0} };//Uchip: B�AG�AR
		//thresParm = { 0,{99999,99999,99999},{99999,99999,99999} ,{255,99999,99999}, {threslist[0],99999,99999} };//Uchip _1020
		//{mode,bgmax,bgmin,fgmax,fgmin}
		//thresParm = { 0,{90,99999,99999},{0,99999,99999} ,{255,9,9}, {120,0,0} };//2040
		thresParm = { 3,{208,99999,99999},{99999,99999,99999} ,{9,99999,99999}, {99999,99999,99999} };
		//thresParm = { 3,{280,99999,99999},{99999,99999,99999} ,{9,99999,99999}, {99999,99999,99999} };//MTchip-R[0]
		//thresParm = { 3,{189,107,107},{99999,99999,99999} ,{3,9,9}, {99999,99999,99999} };//MTchip-LM01P119
		/*create image::::*/
		//CreateRotImg(rawimg, 8280402,-1*imageParm.correctTheta); //negative:counter-clockwise // positive:clockwise

		thresParm.thresmode = 3;
		thresParm.bgmax[0] = 200;
		thresParm.fgmax[0] = 5;

		cropedRImg = CropImgFromChipSetting(rawimg, chipsetting, target, imageParm, boolflag, piccenter, IMGoffset);


		if (boolflag == 0)
		{

			/*Rotate picture::: */
			if (imageParm.correctTheta != 0)
			{
				cropedRImg = RotatecorrectImg(imageParm.correctTheta, cropedRImg);
			}
			/*rotate end----------------*/

			if (imageParm.Outputmode == 0)
			{					
				std::cout << "start to single phase detection...." << endl;
				std::tie(boolflag, ReqIMG, crossCenter, marksize) = Uchip_singlephaseDownV3(boolflag, cropedRImg, thresParm, chipsetting, target, IMGoffset, imageParm);
			}
			else
			{
				vector<Point> vPt;
				PairChip_Finder(boolflag, cropedRImg,ReqIMG,marksize, thresParm, chipsetting, target, crossCenter, IMGoffset, imageParm, vPt);

				if (vPt.size() > 0)
					for (int i = 1; i < sizeof(outputLEDX) / sizeof(outputLEDX[0]); i++)
					{
						if (i-1== vPt.size())
							break;
						outputLEDX[i] = vPt[i-1].x;
						outputLEDY[i] = vPt[i-1].y;
					}

			}

		}

		std::cout << "check img state:: " << boolflag << endl;
		std::cout << "check center is ::" << crossCenter << endl;
	
	return 0;
}

























