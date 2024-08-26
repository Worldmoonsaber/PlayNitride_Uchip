
#include "MTchip_lib_V1.h"


void CreateRotImg(Mat src, int picsavenumber, double theta)
{
	/*create rotate image::::*/
	Point2f center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
	// using getRotationMatrix2D() to get the rotation matrix
	Mat rotation_matix = getRotationMatrix2D(center, theta, 1.0);
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

cv:imwrite(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\)x" + string(std::to_string(picsavenumber)) + ".bmp", patchrotaIMG);
}
std::tuple < vector<float>, vector<int>> dict_rectregion(int picorder)
{
	vector<float> sizelist;
	vector<int> threslist;

	map <int, vector<float>> map_size =
	{


		{120502, vector<float>{408,1.1,0.9,236,1.1,0.7}},
		{204400, vector<float>{335,1.1,0.9,152,1.1,0.7}},
		{34585000, vector<float>{442,1.1,0.9,252,1.1,0.7}},
	};

	map <int, vector<int>> map_thres =
	{
		{829070, vector<int>{220, 100}}, //max,min
		//{34585000, vector<int>{100, 40}},


	};

	if (map_size.count(picorder) == 1)
	{
		sizelist = map_size.find(picorder)->second;
	}
	else
	{
		//sizelist = vector<float>{ 174, 1.1, 0.9, 86, 1.1, 0.7 }; //2040-2044 //335,152
		//sizelist = vector<float>{ 442, 1.1, 0.9, 252, 1.1, 0.7 }; //3458
		//sizelist = vector<float>{ 152, 1.1, 0.9, 335, 1.1, 0.7 }; //vertical2040-2044
		//sizelist = vector<float>{ 252, 1.1, 0.9, 442, 1.1, 0.7 }; //vertical3458
		//sizelist = vector<float>{ 442, 1.1, 0.9, 252, 1.1, 0.7 }; //testl3458
		//sizelist = vector<float>{ 160, 1.1, 0.9, 78, 1.1, 0.7 }; //L5_5x2040  167, 1.1, 0.9, 78, 1.1, 0.7
		//sizelist = vector<float>{ 461,1.2, 0.8, 276, 1.2, 0.8 }; //MT-chip (B&G)
		//sizelist = vector<float>{ 398,1.2, 0.8, 202, 1.2, 0.8 }; //MT-chip (R)
		//sizelist = vector<float>{ 188, 1.1, 0.9, 104, 1.1, 0.7 };  //L5_71701
		//sizelist = vector<float>{ 188, 1.25, 0.9, 110, 1.1, 0.7 };  //L5_72401
		//sizelist = vector<float>{ 135, 1.25, 0.9, 75, 1.5, 0.7 };  //L5_828
		//sizelist = vector<float>{ 135, 1.25, 0.7, 75, 1.5, 0.7 }; //L4_1011
		//sizelist = vector<float>{ 75, 1.5, 0.7, 135, 1.25, 0.7 };//L4_1020
		//sizelist = vector<float>{ 466,1.2, 0.8, 284, 1.2, 0.8 }; //LS01 (B&G)
		//sizelist = vector<float>{ 156,1.2, 0.8, 231, 1.2, 0.8 }; //LS01 (B&G)  200,233P118 / 111,231P119
		//sizelist = vector<float>{ 179,1.2, 0.8, 375, 1.2, 0.8 }; //MT-chip (1by1)
		//sizelist = vector<float>{ 108,1.3, 0.7, 243, 1.3, 0.7 }; //MT-chip (LM01-P119)
		//sizelist = vector<float>{ 488,1.2, 0.8, 301, 1.2, 0.8 };
		//sizelist = vector<float>{ 290,1.2, 0.8, 149, 1.2, 0.8 }; //MT1:1328G
		//sizelist = vector<float>{ 300,1.2, 0.8, 153, 1.2, 0.8 };
		//sizelist = vector<float>{ 125,1.2, 0.8, 236, 1.2, 0.8 };
		//sizelist = vector<float>{ 460,1.2, 0.8, 272, 1.2, 0.8 }; //SR1-88
		//sizelist = vector<float>{ 155,1.2, 0.8, 203, 1.2, 0.8 }; //SR1-118
		//sizelist = vector<float>{ 124,1.2, 0.8, 227, 1.2, 0.8 }; //SR1-11901
		sizelist = vector<float>{ 211, 1.3, 0.7, 231, 1.3, 0.7 };  //LS01-0801
	}


	if (map_thres.count(picorder) == 1)
	{
		threslist = map_thres.find(picorder)->second;

	}
	else
	{
		//threslist = vector<int>{ 67,40 }; //2040-2044  max,min 67,40 ;wierd value=153, 50
		//threslist = vector<int>{ 50, 99999 }; //3458
		//threslist = vector<int>{ 67,40 }; //vertical2040-2044
		//threslist = vector<int>{ 50, 99999 }; //vertical3458
		//threslist = vector<int>{ 92, 25 }; //L5-5x 60-25
		//threslist = vector<int>{ 70, 28 }; //L5-5x 
		//threslist = vector<int>{ 92, 35 };//L5_5x2040
		//threslist = vector<int>{ 140, 85 };//L5_71701 //85
		//threslist = vector<int>{ 155, 95 };//L5_72401
		//threslist = vector<int>{ 110, 99999 };//L5_828
		threslist = vector<int>{ 80, 99999 };//L4_1011
		//threslist = vector<int>{ 100, 99999 };//L4_11


		//threslist = vector<int>{ 120, 55 }; //L5-5x -offset
	}




	return{ sizelist,threslist };
}
std::tuple<int, Mat> Inputfunction()
{
	int picorder, picmode, picseq;

	bool valid = false;
	Mat Rawpic;

	string picseqNAME = "";

	do
	{

		do
		{
			std::cout << "picture sequence: 0(testpic)  / 1(L4-11/10) / 2(MT-chip) /3(LM01) / 4 (Focus) / 5(MT1) / 6(711LS01) /7(SR0718) / 8(LS0801): ";
			std::cin >> picseq;
			if (picseq >-1 && picseq<11)
			{
				valid = true;

				picseqNAME = "/";




			}
			else
			{
				std::cout << "picture sequence: 0(testpic) :";
				std::cin >> picseq;
			}
		} while (!valid);

		if (picseqNAME != "")
		{
			std::cout << "Enter the number of the picture:";
			std::cin >> picorder;
			if (picseq == 0)
			{
				Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image
				//Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\TPlab240520P118P119)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image
			}
			else if (picseq == 1)
			{

				Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\1110)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image
			}
			else if (picseq == 2)
			{
				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\24022729\0229)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

			
			}
			else if (picseq == 3)
			{
				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\24022729\0321)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image


			}
			else if (picseq == 4)
			{
				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\ZCtunning)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image


			}
			else if (picseq == 5)
			{
				//Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\ChipNKey_Keypitch26.5\R_UJ2200380_chip1530PN070_keypitch26.637\chip)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

				Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\240522_MT1Ukey_MT1Uchip1836\240522_MT1Ukey_MT1Uchip1836\chip)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

			}
			else if (picseq == 6)
			{
				//Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\ChipNKey_Keypitch26.5\R_UJ2200380_chip1530PN070_keypitch26.637\chip)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\20240711)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

			}
			else if (picseq == 7)
			{
				//Rawpic = imread(R"x(D:\AOI_Project\L5-1AOI\AUO5xUchip\pic\ChipNKey_Keypitch26.5\R_UJ2200380_chip1530PN070_keypitch26.637\chip)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\Uchip\img\SR1_0718)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

			}
			else if (picseq == 8)
			{
				
				Rawpic = imread(R"x(D:\AOI_Project\L0_MassTrimming\img\0801)x" + picseqNAME + string(std::to_string(picorder)) + ".bmp");//loading the image

			}
			

		
		
		
		

		}
		else
		{
			std::cout << "Error picture sequence" << endl;
		}




	} while (!valid);

	return { picorder ,Rawpic };

}