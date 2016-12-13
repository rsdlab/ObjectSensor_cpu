#include "ImageViewer.h"


namespace ALTH{

	const char* getLabel(pixel_t pixel){
		switch(pixel){
		case PIXEL_UNKNOWN: return "PIXEL_UNKNOWN";// default value (not meant for use)
		case PIXEL_GRAY: return "PIXEL_GRAY"; 	   // grayscale		(1 channel /pixel) (the smallest)
		case PIXEL_RGB: return "PIXEL_RGB";  	   // RGB			(3 channels/pixel)
		case PIXEL_FLOAT: return "PIXEL_FLOAT";    // float values		(1 channels/pixel)
		case PIXEL_VOIDP: return "PIXEL_VOIDP";    // void pointer		(1 channels/pixel)
		}
		return NULL;
	}


	//============================================================
	// ImageViewer class
	//
	// @author H.Takahashi
	// @last modified in July., 2011
	//
	// @brief this class shows image buffer.
	//
	// this class depends on OpenCV library.
	// link "cv.lib" "cvcam.lib" "highgui.lib" "cxcore.lib"
	//============================================================
	// constructor
	ImageViewer::ImageViewer()
	:_wndname("image window"), _height(0), _width(0),
	_imgbuf(0), _active(false)
	{
	}

	// destructor
	ImageViewer::~ImageViewer()
	{
		// destroy window
		closeWindow();
		_imgbuf = 0;
	}

	// open window
	bool ImageViewer::openWindow(){
		if(!_active){
			// create window
			cvNamedWindow(_wndname.c_str(), 0);
			cvResizeWindow(_wndname.c_str(), _width, _height);
			// create iplimage buffer
			createImageBuffer( _width,_height);
			_active = true;
			return true;
		}
		return false;
	}
	// open window
	// @param wndname: window's name
	bool ImageViewer::openWindow(string wndname){
		setWindowName(wndname);
		return openWindow();
	}
	// open window
	// @param wndname: window's name
	//        width: window's width
	//        height: widow's height
	bool ImageViewer::openWindow(string wndname, int width, int height){
		setWindowName(wndname);
		setWindowSize(width, height);
		return openWindow();
	}

	// close window
	bool ImageViewer::closeWindow(){
		if(_active){
			cvDestroyWindow(_wndname.c_str());
			releaseImageBuffer();
			_active = false;
			return true;
		}
		return false;
	}

	// set window's name
	bool ImageViewer::setWindowName(string name){
		if(_active){
			return false;
		}
		_wndname = name;
		return true;
	}

	// set window's size (same image's size)
	// @param width: width of window
	//        height: height of window
	bool ImageViewer::setWindowSize(int width, int height){
		_width = width;
		_height = height;
		cvResizeWindow(_wndname.c_str(), _width, _height);

		return true;
	}
	// set pixel type of image
	// @param ptype: pixel type
	void ImageViewer::setImagePixelType(pixel_t ptype){
		_ptype = ptype;
	}

	// create image buffer
	bool ImageViewer::createImageBuffer(int width, int height){
		int channels(3);
		int depth(IPL_DEPTH_8U);
		if(_ptype == PIXEL_FLOAT){
			channels = 1; depth = IPL_DEPTH_32F;
		}
		_cvimg = cvCreateImage( cvSize(width,height),depth,channels );

		return true;
	}

	// delete image buffer
	bool ImageViewer::releaseImageBuffer(){
		cvReleaseImage( &_cvimg);
		return true;
	}

	// set image buffer
	bool ImageViewer::setImageBuffer(void* imgbuf){
		_imgbuf = imgbuf;
		return true;
	}

	// show image buffer
	bool ImageViewer::display(){
		if(!_active){
			return false;
		}
		if(_imgbuf==NULL){
			return false;
		}
		//// create IPL image for openCV
		//IplImage cvimg;
		int channels(3);
		//int depth(IPL_DEPTH_8U);
		if(_ptype == PIXEL_FLOAT){
			channels = 1;
		}
		//cvInitImageHeader( &cvimg, cvSize(_width, _height), depth, channels, IPL_ORIGIN_TL, 4);
		//_cvimg.imageData = (char*)_imgbuf;
		memcpy( _cvimg->imageData, _imgbuf, _width*_height*channels);
		unsigned char temp;
		if( _ptype == PIXEL_RGB){
			for( int i=0; i< _width*_height;i++){
				//_cvimg->imageData[3*i]   = (char *)_imgbuf[3*i+2];
				//_cvimg->imageData[3*i+1] = (char *)_imgbuf[3*i+1];
				//_cvimg->imageData[3*i+2] = (char *)_imgbuf[3*i];
				temp                     = _cvimg->imageData[3*i];
				_cvimg->imageData[3*i]   = _cvimg->imageData[3*i+2];
				_cvimg->imageData[3*i+2] = temp;

			}
		}

		// show image
		cvShowImage(_wndname.c_str(), _cvimg);

		return true;
	}

	// show image buffer
	// @param imgbuf: pointer to image buffer
	bool ImageViewer::display(void* imgbuf){
		// set image's buffer
		setImageBuffer(imgbuf);
		return display();
	}

	// show image buffer and wait key
	// @param msec: time for waiting[msec]
	//        img : pointer to image buffer
	int ImageViewer::waitKeyDisplay(int msec){
		display();
		return cvWaitKey(msec);
	}
	int ImageViewer::waitKeyDisplay(void* imgbuf, int msec){
		display(imgbuf);
		return cvWaitKey(msec);
	}
}; // namespace ALTH

