#include "MTchip_lib_V1.h"
#include "OpenCV_Extension_Tool.h"


std::tuple<int, Mat, Point, Mat>Uchip_singlephaseDownV3(int flag, Mat stIMG, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point2f creteriaPoint, Point IMGoffset, ImgP_ imageParm)
{
	auto t_start = std::chrono::high_resolution_clock::now();
	Point2f piccenter;
	Point crossCenter;
	Point crossCenternew;
	Mat comthresIMG;

	// Step 1 & Step 2 & Step 3
	funcThreshold(stIMG, comthresIMG, thresParm, imageParm, target);
	Mat Reqcomthres = Mat::zeros(stIMG.rows, stIMG.cols, CV_8UC1);
	//output parameters:::
	Mat marksize;
	stIMG.copyTo(marksize);
	//Step.4 Featurize pattern via dimension filtering
	piccenter = find_piccenter(comthresIMG);
	vector<BlobInfo> blobRegion = RegionPartition(comthresIMG, target.TDwidth * target.TDmaxW * target.TDheight * target.TDmaxH*2, 1);
	Reqcomthres = comthresIMG.clone();

	try
	{
		if (blobRegion.size() == 0)
		{
			flag = 1;
			throw "something wrong::threshold value issue";
		}
		else
		{
			vector<vector<cv::Point>> vContour;
			vector<vector<cv::Point>> vContourFiltered;
			vector<BlobInfo> blobRegionPossible;

			for (int i = 0; i < blobRegion.size(); i++)
			{
				int ww = blobRegion[i].Xmax() - blobRegion[i].Xmin();
				int hh = blobRegion[i].Ymax() - blobRegion[i].Ymin();

				if (ww > target.TDwidth * target.TDmaxW)
				{
					vContourFiltered.push_back(blobRegion[i].Points());
					continue;
				}

				if (hh > target.TDheight * target.TDmaxH)
				{
					vContourFiltered.push_back(blobRegion[i].Points());
					continue;
				}

				if (ww > target.TDwidth * target.TDminW
					&& hh > target.TDheight * target.TDminH
					&& ww < target.TDwidth * target.TDmaxW
					&& hh < target.TDheight * target.TDmaxH
					)
				{
					blobRegionPossible.push_back(blobRegion[i]);
					cv::rectangle(marksize, Point(blobRegion[i].Xmin(), blobRegion[i].Ymin()), Point(blobRegion[i].Xmax(), blobRegion[i].Ymax()), Scalar(255, 255, 255), 1);
					cv::rectangle(Reqcomthres, Point(blobRegion[i].Xmin(), blobRegion[i].Ymin()), Point(blobRegion[i].Xmax(), blobRegion[i].Ymax()), Scalar(255, 255, 255), -1);
				}
				else
					vContour.push_back(blobRegion[i].Points());

			}

			//draw pic center:: 
			cv::circle(marksize,
				Point2i(piccenter), //coordinate
				9, //radius
				Scalar(0, 255, 255),  //color
				FILLED,
				LINE_AA);

			if (blobRegionPossible.size() == 0)
			{
				flag = 2;
				throw "something wrong::potential object doesn't fit suitable dimension";
			}
			else
			{

				std::sort(blobRegionPossible.begin(), blobRegionPossible.end(), [&, piccenter](BlobInfo& a, BlobInfo& b)
					{
						norm(a.Center() - piccenter);
						return norm(a.Center() - piccenter) < norm(b.Center() - piccenter);
					});

				float dist = norm(blobRegionPossible[0].Center() - piccenter);

				if ( dist> chipsetting.xpitch[0])
				{
					flag = 6;
					throw "something wrong::potential object doesn't fit suitable dimension";
				}
				else
				{
					crossCenter = blobRegionPossible[0].Center() + Point2f(chipsetting.carx, chipsetting.cary);

					cv::circle(marksize,
						(Point2i(crossCenter)), //coordinate
						6, //radius
						Scalar(255, 0, 255),  //color
						FILLED,
						LINE_AA);

					cv::line(marksize, Point(0, crossCenter.y), Point(marksize.size[1], crossCenter.y), Scalar(255, 255, 255), 1, 8);
					cv::line(marksize, Point(crossCenter.x, 0), Point(crossCenter.x, marksize.size[0]), Scalar(255, 255, 255), 1, 8);

					flag = 9;


					if (imageParm.correctTheta != 0)
					{
						vector<Point> vPt;
						vPt.push_back(crossCenter);

						vector<Point> vPtOut;
						funcRotatePoint(vPt, vPtOut, marksize, imageParm.correctTheta, IMGoffset);

						if (vPtOut.size() > 0)
							crossCenternew = vPtOut[0];
					}
					else
					{
						crossCenternew = crossCenter + IMGoffset;
						std::cout << "check chip crossCenternew is: [ " << crossCenternew << " ]" << endl;
						std::cout << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << endl;
					}

				}


			}


		}// if-loop:contH fits suitable area 



	} //try-loop
	catch (const char* message)
	{
		std::cout << message << std::endl;

	}


	/*Mat Rotmark;
	Rotmark = RotatecorrectImg(-2.6, marksize);*/
	comthresIMG.release();
	stIMG.release();
	//////////////////////////////////////////////////////////output//////////////////////////////////
	auto t_end = std::chrono::high_resolution_clock::now();
	double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cout << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << endl;
	std::cout << "check chip center is: [ " << crossCenter << " ]" << endl;
	std::cout << "result flag is :: " << flag << endl;
	std::cout << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << endl;
	std::cout << "calculate color-filter time is:: " << elapsed_time_ms << endl;

	std::cout << "fini" << endl;
	return { flag, Reqcomthres, crossCenternew, marksize };
}




