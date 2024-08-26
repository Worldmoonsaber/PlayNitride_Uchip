#include "MTchip_lib_V1.h"

/*Input function*/
std::tuple<int, Mat> Inputfunction();
std::tuple < vector<float>, vector<int>> dict_rectregion(int picorder);
void CreateRotImg(Mat src, int picsavenumber, double theta);