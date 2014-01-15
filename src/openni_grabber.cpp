//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/image_viewer.h>
 
//#include <cv.h>
//#include <cvaux.h>
//#include <cxcore.h>
//#include <highgui.h>
//#include <cxmisc.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mandelbrot.hh"


static cv::Mat *img;



/*
class SimpleOpenNIViewer
{
	public:
		SimpleOpenNIViewer () {}

		void image_cb (const openni_wrapper::Image::Ptr &openni_img)
		{
			cv::imgshow( "Window", *img);
			
			//	if (!i_viewer.wasStopped())
			//	{
			//		i_viewer.addRGBImage(myimg, 640, 480);
			//		i_viewer.spin();
			//		unsigned char* data = new unsigned char[img->getWidth()*img->getHeight()*3];
			//		img->fillRGB(img->getWidth(), img->getHeight(), data);
			//		i_viewer.addRGBImage(data, img->getWidth(), img->getHeight());
			//		delete [] data;
			//	}
		}

		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();
			
			boost::function<void (const openni_wrapper::Image::Ptr&)> f = boost::bind (&SimpleOpenNIViewer::image_cb, this, _1);
			interface->registerCallback (f);
		     
			interface->start();

			while (1)
				boost::this_thread::sleep (boost::posix_time::seconds (1));

			interface->stop();
		}
};
*/

int main ()
{ 
	img = new cv::Mat(600, 800, CV_8UC3);
	makeMandelbrot(*img, -0.74, -0.2, 5e-5);

	cv::imshow("Mandelbrot", *img);
	cv::waitKey(0);
			
//	SimpleOpenNIViewer v;
//	v.run ();
	
	return 0;
}
