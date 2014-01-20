//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/image_viewer.h>
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat* img;

class SimpleOpenNIViewer
{
	public:
		SimpleOpenNIViewer () {}

		void image_cb (const openni_wrapper::Image::Ptr &openni_img)
		{
			cv::imgshow( "Window", *img);
			
			if (!i_viewer.wasStopped())
			{
				i_viewer.addRGBImage(myimg, 640, 480);
				i_viewer.spin();
			
				unsigned char* data = new unsigned char[img->getWidth()*img->getHeight()*3];
				img->fillRGB(img->getWidth(), img->getHeight(), data);
				i_viewer.addRGBImage(data, img->getWidth(), img->getHeight());
				delete [] data;
			
			}
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


int main ()
{ 
	img = new cv::Mat(640, 480, CV_8UC3, cv::Scalar(1, 0, 0));

	cv::imshow("Mandelbrot", *img);
	cv::waitKey(0);
			
	//SimpleOpenNIViewer v;
	//v.run ();
	
	return 0;
}
