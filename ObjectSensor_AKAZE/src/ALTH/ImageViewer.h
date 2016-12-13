#ifndef _IMAGE_VIEWER_H_
#define _IMAGE_VIEWER_H_

//
// Image Viewer Class
//
// @author H.Takahashi
//
// @last modified in Aug., 2009
//


#include<string>
//============================================================
// OpenCV include
//============================================================
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace std;

namespace ALTH{

	#ifndef _PIXEL_TYPE_
	#define _PIXEL_TYPE_
	typedef enum {
		PIXEL_UNKNOWN=0,// default value (not meant for use)
		PIXEL_GRAY, 	// grayscale		(1 channel /pixel) (the smallest)
		PIXEL_RGB,  	// RGB			(3 channels/pixel)
		PIXEL_FLOAT,  	// float values		(1 channels/pixel)
		PIXEL_VOIDP,  	// void pointer		(1 channels/pixel)
	}  pixel_t;

	const char* getLabel(pixel_t pixel);
	#endif

	//==============================================================
	// ImageViewer class
	//
	// @brief This class shows image
	// eg.1 setWindowName->setWindowSize->openWindow
	//     ->waitKeyDisplay->closeWindow
	// eg.2 openWindow(name, 640,480)->waitKeyDisplay->closeWindow
	//==============================================================
	class ImageViewer {
	private:
		// window's name
		string _wndname;

		// widow size
		int _height;
		int _width;

		// window' status
		// true:(active), false:(deactive)
		bool _active;

		// pointer to image buffer
		void* _imgbuf;

		// pixel type
		pixel_t _ptype;

		// Iplimage buffer
		IplImage *_cvimg;

	public:
		// constructor
		ImageViewer();

		// destructor
		~ImageViewer();

		// set window's name
		bool setWindowName(string name);

		// set window's size (same image's size)
		// @param width: width of window
		//        height: height of window
		bool setWindowSize(int width, int height);

		// set pixel type of image
		// @param ptype: pixel type
		void setImagePixelType(pixel_t ptype);

		// create image buffer
		bool createImageBuffer(int width, int height);

		// release image buffer
		bool releaseImageBuffer();

		// set image buffer
		bool setImageBuffer(void* imgbuf);

		// show image buffer
		bool display();
		// show image buffer
		// @param imgbuf: pointer to image buffer
		bool display(void* imgbuf);

		// show image buffer and wait key
		// @param msec: time for waiting[msec]
		//        img : pointer to image buffer
		int waitKeyDisplay(int msec);
		int waitKeyDisplay(void* imgbuf, int msec);


		// open window
		bool openWindow();
		// open window
		// @param wndname: window's name
		bool openWindow(string wndname);
		// open window
		// @param wndname: window's name
		//        width: window's width
		//        height: widow's height
		bool openWindow(string wndname, int width, int height);

		// close window
		bool closeWindow();
	};

}; // namespace ALTH
#endif //_IMAGE_VIEWER_H_
