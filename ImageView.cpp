#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) 
{
	namedWindow("ImageView", CV_WINDOW_AUTOSIZE);
	
	while(true)
	{
		try {
			Mat image = imread("outImage.jpg",0);	
			if((image.cols > 0) & (image.rows > 0))
				imshow("ImageView", image);
		}
		catch(runtime_error& ex){
			//Ignore
		}
		
		int i = waitKey(1);
		if( i > 0 )
			break;
			
	}
}
