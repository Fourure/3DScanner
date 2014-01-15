#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
 
#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#include <cxmisc.h>

static unsigned char myimg[640*480*3];
static cv::Mat *img;


 class SimpleOpenNIViewer
 {
	public:
		SimpleOpenNIViewer () {}

		void image_cb (const openni_wrapper::Image::Ptr &openni_img)
		{
			//cv::imgshow( "Window", *img);
			
			/*if (!i_viewer.wasStopped())
			{
				i_viewer.addRGBImage(myimg, 640, 480);
				i_viewer.spin();
			
				unsigned char* data = new unsigned char[img->getWidth()*img->getHeight()*3];
				img->fillRGB(img->getWidth(), img->getHeight(), data);
				i_viewer.addRGBImage(data, img->getWidth(), img->getHeight());
				delete [] data;
			
			}
			*/
		}

		void run ()
		{
			pcl::Grabber* interface= new pcl::OpenNIGrabber();
			
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
	
	for (int i = 0; i<640; ++i)
		for (int j = 0; j<480; ++j)
			for (int c = 0; c<3; ++c)
				myimg[(i*480+j)*3+c] = (unsigned char) ( 255.0 * sin(7.0*i + 5.0*j) );
	img = new cv::Mat(480, 640, CV_8UC3, myimg);
	
	namedWindow("OpenCV");
	
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}
