#include "MTchip_lib_V1.h"


std::tuple<int, Mat, Point, Mat>Uchip_singlephaseDownV3(int flag, Mat stIMG, thresP_ thresParm, SettingP_ chipsetting, sizeTD_ target, Point2f creteriaPoint, Point IMGoffset, ImgP_ imageParm)
{
	Mat Reqcomthres = Mat::zeros(stIMG.rows, stIMG.cols, CV_8UC1);
	Point2f piccenter;
	Point crossCenter;
	Mat marksize;
	Mat Rotmarkpic = Mat::ones(stIMG.rows, stIMG.cols, CV_8UC3);
	Mat Rotnew = Mat::ones(stIMG.rows, stIMG.cols, CV_8UC3);
	Mat thresRot;
	Point crossCenternew;
	stIMG.copyTo(marksize);

	//Thres parameters:::
	Mat Gimg, gauGimh;
	Mat gauBGR;
	vector<vector<Point>>  contH, contRot;
	vector<Vec4i> hierH, hierRot;
	vector<vector<Point>> reqConH;

	Mat adptThres;
	Mat HIGHthres;
	Mat comthresIMG;
	int adaptWsize = 3;
	int adaptKsize = 2;

	vector<vector<Point>>  contours, REQcont;
	Rect retCOMP;
	vector<Rect> Rectlist;
	vector<Point2f> center;
	vector<double> distance;
	int minIndex;
	vector<Point> approx;
	vector< double> approxList;
	double areacomthres;






	auto t_start = std::chrono::high_resolution_clock::now();


	// Step 1 & Step 2 & Step 3
	funcThreshold(stIMG, comthresIMG, thresParm, imageParm, target);
	//output parameters:::

	//Step.4 Featurize pattern via dimension filtering
	Mat finescanIMG = Mat::zeros(stIMG.rows, stIMG.cols, CV_8UC1);
	Rect drawrect;
	Rect fineRect;
	Point centerTD;


	cv::findContours(comthresIMG, contH, hierH,
		cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	cv::drawContours(Reqcomthres, contH, -1, Scalar(255, 255, 255), -1);


	try
	{

		if (contH.size() == 0)
		{
			flag = 1;
			throw "something wrong::threshold value issue";
		}

		else
		{
			if (thresParm.thresmode == 3 || 4)
			{


				for (int i = 0; i < contH.size(); i++)
				{

					retCOMP = cv::boundingRect(contH[i]);
					areacomthres = cv::contourArea(contH[i]);
					cv::approxPolyDP(contH[i], approx, 15, true); 
					if (retCOMP.width > target.TDwidth * target.TDminW
						&& retCOMP.height > target.TDheight * target.TDminH
						&& retCOMP.width < target.TDwidth * target.TDmaxW
						&& retCOMP.height < target.TDheight * target.TDmaxH
						)

					{
						Moments M = (moments(contH[i], false));
						center.push_back((Point2f((M.m10 / M.m00), (M.m01 / M.m00))));
						piccenter = find_piccenter(comthresIMG);
						distance.push_back(norm((creteriaPoint)-center[center.size() - 1])); // get Euclidian distance
						Rectlist.push_back(retCOMP);
						approxList.push_back(approx.size());
						REQcont.push_back(contH[i]);
						cv::rectangle(marksize, retCOMP, Scalar(255, 255, 255), 4);
						cv::rectangle(Reqcomthres, retCOMP, Scalar(255, 255, 255), -1);


					}

				} //for-loop: contours



			}//if-loop: (thresParm.thresmode == 3 || 4)

			else //(thresParm.thresmode == 0 || 1|| 2)
			{
				for (int i = 0; i < contH.size(); i++)
				{

					retCOMP = cv::boundingRect(contH[i]);
					areacomthres = cv::contourArea(contH[i]);
					cv::approxPolyDP(contH[i], approx, 15, true); 
					if (areacomthres > target.TDwidth * target.TDminW * target.TDheight * target.TDminH && areacomthres < target.TDwidth * target.TDmaxW * target.TDheight * target.TDmaxH)

					{
						Moments M = (moments(contH[i], false));
						center.push_back((Point2f((M.m10 / M.m00), (M.m01 / M.m00))));
						piccenter = find_piccenter(comthresIMG);
						distance.push_back(norm((creteriaPoint)-center[center.size() - 1])); // get Euclidian distance
						Rectlist.push_back(retCOMP);
						approxList.push_back(approx.size());
						REQcont.push_back(contH[i]);
						cv::rectangle(marksize, retCOMP, Scalar(255, 255, 255), 4);
						cv::rectangle(Reqcomthres, retCOMP, Scalar(255, 255, 255), -1);


					}

				} //for-loop: contours
			} //if-loop: (thresParm.thresmode == 0 || 1|| 2)


			//draw pic center:: 
			cv::circle(marksize,
				Point2i(piccenter), //coordinate
				9, //radius
				Scalar(0, 255, 255),  //color
				FILLED,
				LINE_AA);

			if (center.size() == 0)
			{
				flag = 2;
				throw "something wrong::potential object doesn't fit suitable dimension";
			}
			else
			{

				//Step.5 Define center chip ::Find a LED coordinate with the shortest distance to the pic center
				auto it = std::min_element(distance.begin(), distance.end());
				minIndex = std::distance(distance.begin(), it);
				

				if (distance[minIndex] > chipsetting.xpitch[0])
				{
					flag = 6;
					throw "something wrong::potential object doesn't fit suitable dimension";
				}
				else
				{
					std::wcout << "check approx  size main: " << approxList[minIndex] << endl;


					if (approxList[minIndex] == 4)
					{
						crossCenter = center[minIndex] + Point2f(chipsetting.carx, chipsetting.cary);
						drawrect = Rect(Rectlist[minIndex].x,
							Rectlist[minIndex].y, //rectangle ini y
							Rectlist[minIndex].width, //rectangle width
							Rectlist[minIndex].height); //rectangle height


					}
					else
					{
						//Step.6 Redefine center coordinate:
						std::cout << "start fine define...." << endl;
						cv::drawContours(finescanIMG, REQcont, minIndex, Scalar(255, 255, 255), -1);

						tie(fineRect, centerTD) = FindMaxInnerRect(finescanIMG, stIMG, target, center[minIndex]);

						crossCenter = Point2f(centerTD) + Point2f(chipsetting.carx, chipsetting.cary);
						drawrect = fineRect;

					}

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




