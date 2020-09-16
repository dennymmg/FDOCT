/*
* modified from BscanFFTspinjnt.cpp
* for FLIR Point Grey camera
* using Spinnaker SDK
* instead of QHY camera,
* doing J0 subtraction
* and batch capture with hardware triggering.
*
* Implementing line scan FFT
* for SD OCT
* with binning
* and inputs from ini file.
*
* Captures (background) J0 null on high level of line0 trigger
* Captures signal image        on  low level  of line0 trigger
* Displays Bscan  batch-averaged difference of Signal and J0 null 
*
* Number of images averaged in each batch is read from the ini file
*
*
* a (or A) key changes to Accumulate mode
* l (or L) key changes back to Live (default) mode
* r (or R) key in accumulate mode clears the accumulated frames so far (Refresh)
* + (or =) key increases exposure time by 0.1 ms
* - (or _) key decreases exposure time by 0.1 ms
* u key increases exposure time by 1 ms
* d key decreases exposure time by 1 ms
* U key increases exposure time by 10 ms
* D key decreases exposure time by 10 ms
* ] key increases thresholding in final Bscan
* [ key decreases thresholding in final Bscan
*  
*
* ESC, x or X key quits
*
*
*
* Denny
* 16 Sep 2019  *
*
*
*/
#include <iostream>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace cv;

void setCamera(CameraPtr pCam)
{
	int result = 0;    
	unsigned int w, h, camspeed, burstframecount,triggerdelay, camtime, camgain = 1, bpp;
	unsigned int offsetx = 0, offsety = 0;
	unsigned int cambinx, cambiny;
	
	ifstream infile("spin.ini");
	string tempstring;
	
	// inputs from ini file
	if (infile.is_open())
	{
		infile >> tempstring;
		infile >> tempstring;
		infile >> tempstring;
		// first three lines of ini file are comments
		infile >> camgain;
		infile >> tempstring;
		infile >> camtime;
		infile >> tempstring;
		infile >> bpp;
		infile >> tempstring;
		infile >> w;
		infile >> tempstring;
		infile >> h;
		infile >> tempstring;
		infile >> offsetx;
		infile >> tempstring;
		infile >> offsety;
		infile >> tempstring;
		infile >> camspeed;
		infile >> tempstring;
		infile >> burstframecount;
		infile >> tempstring;
		infile >> triggerdelay;
		infile >> tempstring;
		infile >> cambinx;
		infile >> tempstring;
		infile >> cambiny;

		infile.close();
	}

	cout << "Initialising Camera settings ..." << endl;
	
	pCam->TLStream.StreamBufferHandlingMode.SetValue(StreamBufferHandlingMode_NewestOnly);
	pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		
	// gain
	pCam->GainAuto.SetValue(GainAuto_Off);	
	pCam->Gain.SetValue(camgain);
	cout << "Gain set to " << pCam->Gain.GetValue() << " dB ..." << endl;

	// exposure time
	pCam->ExposureAuto.SetValue(ExposureAuto_Off);
	pCam->ExposureMode.SetValue(ExposureMode_Timed);
	pCam->ExposureTime.SetValue(camtime);
	cout << "Exp set to " << pCam->ExposureTime.GetValue() << " microsec ..." << endl;

	// bpp or cambitdepth 
	if (bpp == 16)
	{
		pCam->PixelFormat.SetValue(PixelFormat_Mono16);
		cout << "Pixel format set to " << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..." << endl;
	}
	
	// cambinx
	pCam->BinningHorizontal.SetValue(cambinx);
	cout << "BinningHorizontal set to " << pCam->BinningHorizontal.GetValue() << "..." << endl;

	// cambiny
	pCam->BinningVertical.SetValue(cambiny);
	cout << "BinningVertical set to " << pCam->BinningVertical.GetValue() << "..." << endl;
	
	// width 
	if (IsReadable(pCam->Width) && IsWritable(pCam->Width))
	{
		pCam->Width.SetValue(w);
	}
	else
	{
		cout << "Width not available..." << endl;
	}
	
	// height 
	if (IsReadable(pCam->Height) && IsWritable(pCam->Height))
	{
		pCam->Height.SetValue(h);
	}
	else
	{
		cout << "Height not available..." << endl;
	}

	// offsetx
	if (IsReadable(pCam->OffsetX) && IsWritable(pCam->OffsetX))
	{
		pCam->OffsetX.SetValue(offsetx);
	}
	else
	{
		cout << "Offset X not available..." << endl;
	}
	
	// offsety
	if (IsReadable(pCam->OffsetY) && IsWritable(pCam->OffsetY))
	{
		pCam->OffsetY.SetValue(offsety);
	}
	else
	{
		cout << "Offset Y not available..." << endl;
	}

	// frame rate
	pCam->AcquisitionFrameRateEnable.SetValue(1);
	pCam->AcquisitionFrameRate.SetValue(camspeed);
	cout << "Frame rate set to " << camspeed << endl;

	// set the hardware trigger	     
	pCam->TriggerMode.SetValue(TriggerMode_Off);
	pCam->TriggerSelector.SetValue(TriggerSelector_FrameBurstStart);
	pCam->AcquisitionBurstFrameCount.SetValue(burstframecount);
	pCam->TriggerSource.SetValue(TriggerSource_Line0);
	pCam->TriggerActivation.SetValue(TriggerActivation_AnyEdge);
	pCam->TriggerMode.SetValue(TriggerMode_On);
	pCam->TriggerDelay.SetValue(triggerdelay);
	cout << "Camera set to trigger mode ON \n\t with trigger source as Line0, \n\t trigger selector as FrameBurstStart and \n\t AcquisitionBurstFrameCount set  to " << burstframecount << "\n\t Trigger delay set to " << triggerdelay<< endl;

}

inline void makeonlypositive(Mat& src, Mat& dst)
{
	// from https://stackoverflow.com/questions/48313249/opencv-convert-all-negative-values-to-zero
	max(src, 0, dst);

}

int main()
{
	system("clear");
	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	int result = 0, ret;	
	unsigned int w, h;
	int camtime;
	bool expchanged = false, accummode = false, refreshkeypressed = false;
	bool J0Null = true;
	unsigned int numdisplaypoints = 512;
	int key, fps;
	int t_start, t_end;
	bool doneflag = false;	
	int num = 0;
	char lambdamaxstr[40];
	char lambdaminstr[40];
	double lambdamin, lambdamax;

	uint  indextemp, indextempJ, indextempS, averagescount = 1;
	double bscanthreshold = -30.0;
	
	Mat m, mvector, bscantemp, bscan, jscan, jdiff, bscanlog, bscandb, tempmat, bscandisp;
	Mat positivediff;
	Mat statusimg = Mat::zeros(cv::Size(600, 300), CV_64F);
	Mat firstrowofstatusimg = statusimg(Rect(0, 0, 600, 50)); // x,y,width,height
	Mat secrowofstatusimg = statusimg(Rect(0, 50, 600, 50));
	Mat secrowofstatusimgRHS = statusimg(Rect(300, 50, 300, 50));
	char textbuffer[80];

	Mat lambdas, k, klinear;
	Mat diffk, slopes, fractionalk, nearestkindex;

	Mat magI, cmagI;
	
	double kmin, kmax;
	double pi = 3.141592653589793;
	double minVal, maxVal;

	namedWindow("show", 0); // 0 = WINDOW_NORMAL
	moveWindow("show", 0, 0);

	namedWindow("Status", 0); // 0 = WINDOW_NORMAL
	moveWindow("Status", 0, 500);
	
	namedWindow("Bscan", 0); // 0 = WINDOW_NORMAL
	moveWindow("Bscan", 800, 0);

	SystemPtr system = System::GetInstance();
    
	// Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
    
	unsigned int numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl << endl;
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        cout << "Camera not detected. " << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
	
	CameraPtr pCam = nullptr;
	pCam = camList.GetByIndex(0);
	
	ifstream infile("spin.ini");
	string tempstring;
	
	struct tm *timenow;
	time_t now = time(NULL);
	char dirdescr[60];
	sprintf(dirdescr, "_");
	char dirname[80];
	
	// inputs from ini file
	if (infile.is_open())
	{
		// skip the first 26 lines - they contain camera settings
		for ( int i = 0; i < 26; i++)
		{
			infile >> tempstring;
		}

		infile >> tempstring;
		infile >> lambdaminstr;
		infile >> tempstring;
		infile >> lambdamaxstr;
		infile >> tempstring;
		infile >> averagescount;
		infile >> tempstring;
		infile >> numdisplaypoints;
		infile >> tempstring;
		infile >> dirdescr;
		
		infile.close();
		lambdamin = atof(lambdaminstr);
		lambdamax = atof(lambdamaxstr);
		cout << "lambdamin set to " << lambdamin << " ..." <<endl;
		cout << "lambdamax set to " << lambdamax << " ..." << endl;
	}
	try
	{
		// Initialize camera
		pCam->Init();
		
		setCamera(pCam); // set for trigger mode

		w = pCam->Width.GetValue();
		h = pCam->Height.GetValue();
		camtime = pCam->ExposureTime.GetValue();

		Mat J, S;
		
		indextempJ = 0;
		indextempS = 0;
		t_start = time(NULL);
		fps = 0;
		J = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
		S = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
	
		resizeWindow("Bscan", h*3, numdisplaypoints*3);		// (width,height)

		Mat data_y(h, w, CV_64F);		// the Mat constructor Mat(rows,columns,type)
		Mat barthannwin(1, w, CV_64F);	// the Mat constructor Mat(rows,columns,type);
		unsigned int numfftpoints = data_y.cols;	
		Mat data_ylin(h, numfftpoints, CV_64F);
	
		double deltalambda = (lambdamax - lambdamin) / data_y.cols;
	
		// create modified Bartlett-Hann window
		for (uint p = 0; p<(w); p++)
		{
			// https://in.mathworks.com/help/signal/ref/barthannwin.html
			float nn = p;
			float NN = w - 1;
			barthannwin.at<double>(0, p) = 0.62 - 0.48*abs(nn / NN - 0.5) + 0.38*cos(2 * pi*(nn / NN - 0.5));
		}
		
		klinear = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);
		fractionalk = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);
		nearestkindex = Mat::zeros(cv::Size(1, numfftpoints), CV_32S);
		
		lambdas = Mat::zeros(cv::Size(1, data_y.cols), CV_64F);		//Size(cols,rows)
		diffk = Mat::zeros(cv::Size(1, data_y.cols), CV_64F);
		slopes = Mat::zeros(cv::Size(data_y.rows, data_y.cols), CV_64F);
	
		
		// compute lambdas
		for (indextemp = 0; indextemp< (data_y.cols); indextemp++)
		{
			lambdas.at<double>(0, indextemp) = lambdamin + indextemp * deltalambda;
		}
		
		k = 2 * pi / lambdas;
		kmin = 2 * pi / (lambdamax - deltalambda);
		kmax = 2 * pi / lambdamin;
		double deltak = (kmax - kmin) / numfftpoints;
		
		// compute klinear
		for (indextemp = 0; indextemp<(numfftpoints); indextemp++)
		{
			klinear.at<double>(0, indextemp) = kmin + (indextemp + 1)*deltak;
		}
		
		// find the diff of the non-linear ks
		for (indextemp = 1; indextemp<(data_y.cols); indextemp++)
		{
			// since this is a decreasing series, RHS is (i-1) - (i)
			diffk.at<double>(0, indextemp) = k.at<double>(0, indextemp - 1) - k.at<double>(0, indextemp);
		}
		// and initializing the first point separately
		diffk.at<double>(0, 0) = diffk.at<double>(0, 1);
	
		// find the index of the nearest k value, less than the linear k
		for (int f = 0; f < numfftpoints; f++)
		{
			for (indextemp = 0; indextemp < data_y.cols; indextemp++)
			{
				if (k.at<double>(0, indextemp) < klinear.at<double>(0, f))
				{
					nearestkindex.at<int>(0, f) = indextemp;
					break;
				}// end if
			}//end indextemp loop
		}// end f loop
	
		// now find the fractional amount by which the linearized k value is greater than the next lowest k
		for (int f = 0; f < numfftpoints; f++)
		{
			fractionalk.at<double>(0, f) = (klinear.at<double>(0, f) - k.at<double>(0, nearestkindex.at<int>(0, f))) / diffk.at<double>(0, nearestkindex.at<int>(0, f));
		}

		timenow = localtime(&now);
		strftime(dirname, sizeof(dirname), "%Y-%m-%d_%H_%M_%S-", timenow);
		strcat(dirname, dirdescr);
		// cout << "dir name:" << dirname << endl;
		
		
		cout << "Acquiring images " << endl;
		pCam->BeginAcquisition();
		ImagePtr pResultImage;
		ImagePtr convertedImage;
		while (1)	//camera frames acquisition loop, which is inside the try
		{
			ret = 0;
			while(ret == 0)
			{
				pResultImage = pCam->GetNextImage();
				if (pResultImage->IsIncomplete())
				{
					ret = 0;
				}
				else
				{
					ret = 1;
					num ++;
					convertedImage = pResultImage;
					//Mono16 w x h
					m = cv::Mat(h, w, CV_16UC1, convertedImage->GetData(), convertedImage->GetStride());
				}
			}
	    	// pResultImage has to be released to avoid buffer filling up
			pResultImage->Release();

			if(ret == 1)
			{
				m.convertTo(data_y, CV_64F);	// initialize data_y
				// DC removal and windowing
				for (int p = 0; p<(data_y.rows); p++)
				{
					Scalar meanval = mean(data_y.row(p));
					data_y.row(p) = data_y.row(p) - meanval(0);		// Only the first value of the scalar is useful for us
					multiply(data_y.row(p), barthannwin, data_y.row(p));
				}
				// interpolate to linear k space
				for (int p = 0; p<(data_y.rows); p++)
				{
					for (int q = 1; q<(data_y.cols); q++)
					{
						//find the slope of the data_y at each of the non-linear ks
						slopes.at<double>(p, q) = data_y.at<double>(p, q) - data_y.at<double>(p, q - 1);
						// in the .at notation, it is <double>(y,x)
					}
					// initialize the first slope separately
					slopes.at<double>(p, 0) = slopes.at<double>(p, 1);

					for (int q = 1; q<(data_ylin.cols - 1); q++)
					{
						//find the value of the data_ylin at each of the klinear points
						// data_ylin = data_y(nearestkindex) + fractionalk(nearestkindex)*slopes(nearestkindex)
						data_ylin.at<double>(p, q) = data_y.at<double>(p, nearestkindex.at<int>(0, q))
							+ fractionalk.at<double>(nearestkindex.at<int>(0, q))
							* slopes.at<double>(p, nearestkindex.at<int>(0, q));
					}
				}
				// InvFFT
				Mat planes[] = { Mat_<float>(data_ylin), Mat::zeros(data_ylin.size(), CV_32F) };
				Mat complexI;
				merge(planes, 2, complexI);       // Add to the expanded another plane with zeros
				dft(complexI, complexI, DFT_ROWS | DFT_INVERSE);
				split(complexI, planes);          // planes[0] = Re(DFT(I)), planes[1] = Im(DFT(I))
				magnitude(planes[0], planes[1], magI);
				bscantemp = magI.colRange(0, numdisplaypoints);
				bscantemp.convertTo(bscantemp, CV_64F);

				if (J0Null == true && indextempJ < averagescount)
				{
					// accumulate
					accumulate(bscantemp, J);
					indextempJ++;
				}
				if (indextempJ >= averagescount)
				{
					J0Null = false;	
				}
				if (J0Null == false && indextempS < averagescount)
				{
					// accumulate
					accumulate(bscantemp, S);
					indextempS++;
				}
				if (indextempS >= averagescount)
				{
					J0Null = true;
				}
				if (indextempJ >= averagescount && indextempS >= averagescount)
				{
					sprintf(textbuffer, "%03d images acq.", indextemp);
					secrowofstatusimgRHS = Mat::zeros(cv::Size(300, 50), CV_64F);
					putText(statusimg, textbuffer, Point(300, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					indextempJ = 0;
					indextempS = 0;

					// compute Bscan

					transpose(J, jscan);
					jscan = jscan / averagescount;

					transpose(S, bscan);
					bscan = bscan / averagescount;

					jdiff = bscan - jscan;	// these are in linear scale
					jdiff.copyTo(positivediff);		// just to initialize the Mat
					makeonlypositive(jdiff, positivediff);
					positivediff += 0.0000000001;			// to avoid log(0)
				
					log(positivediff, bscanlog);				// switch to logarithmic scale
					bscandb = 20.0 * bscanlog / 2.303;
					bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
					bscandb.row(4).copyTo(bscandb.row(0));
					tempmat = bscandb.rowRange(0, numdisplaypoints);
					tempmat.copyTo(bscandisp);
					bscandisp = max(bscandisp, bscanthreshold);
					normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
					bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
					applyColorMap(bscandisp, cmagI, COLORMAP_JET);
					
					imshow("Bscan", cmagI);
					
					if (accummode == true || refreshkeypressed == false)
					{
						J = jscan;
						S = bscan;
					}
					else
					{
						J = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
						S = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
					}
					if (accummode == true && refreshkeypressed == true)
					{
						refreshkeypressed = false;
					}

				}
				// update the image windows
				t_end = time(NULL);
				fps++;
				if (t_end - t_start >= 1)  // update every second
				{
					m.copyTo(mvector);
					mvector.reshape(0, 1);	//make it into a row array
					minMaxLoc(mvector, &minVal, &maxVal);
					sprintf(textbuffer, "fps = %d   Max Intensity = %d", fps, int(floor(maxVal)));
					firstrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
					putText(statusimg, textbuffer, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					fps = 0;
					t_start = time(NULL);
					
					resizeWindow("show", w, h);
					imshow("show", m);
					
					resizeWindow("Status", 600, 300);
					imshow("Status", statusimg);
					
				}
				// cout << "number of images acquired: " << num << endl;

				key = waitKey(1); // wait for keypress
				switch (key)
				{

				case 27: //ESC key
				case 'x':
				case 'X':
					doneflag = true;
					break;
				
				case '+':
				case '=':
					camtime = camtime + 100;
					expchanged = true;
					break;

				case '-':
				case '_':
					if (camtime < 8)	// spinnaker has a min of 8 microsec
					{
						camtime = 8;
						break;
					}
					camtime = camtime - 100;
					expchanged = true;
					break;
				
				case 'u':
					camtime = camtime + 1000;
					expchanged = true;
					break;

				case 'd':
					if (camtime < 8)	// spinnaker has a min of 8 microsec
					{
						camtime = 8;
						break;
					}
					camtime = camtime - 1000;
					expchanged = true;
					break;

				case 'U':
					camtime = camtime + 10000;
					expchanged = true;
					break;

				case 'D':
					if (camtime < 8)	// spinnaker has a min of 8 microsec
					{
						camtime = 8;
						break;
					}
					camtime = camtime - 10000;
					expchanged = true;
					break;

				case ']':
					bscanthreshold += 1.0;
					sprintf(textbuffer, "bscanthreshold = %f", bscanthreshold);
					secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
					putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					imshow("Status", statusimg);
					break;

				case '[':
					bscanthreshold -= 1.0;
					sprintf(textbuffer, "bscanthreshold = %f", bscanthreshold);
					secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
					putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					imshow("Status", statusimg);
					break;
			
				case 'a':
				case 'A':
					// accumulate mode
					accummode = true;
					break;

				case 'l':
				case 'L':
					// live mode
					accummode = false;
					break;

				case 'r':
				case 'R':
					// refresh key pressed
					refreshkeypressed = true;
					break;

				default:
					break;
				}
	
				if (doneflag == 1)
				{
					break;
				}

				if (expchanged == true)
				{
					//Set exp with QuickSpin
					ret = 0;
					if (IsReadable(pCam->ExposureTime) && IsWritable(pCam->ExposureTime))
					{
						pCam->ExposureTime.SetValue(camtime);
						ret = 1;
					}
					if (ret == 1)
					{
						sprintf(textbuffer, "Exp time = %d ", camtime);
						secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
						putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
						imshow("Status", statusimg);

					}
					else
					{
						sprintf(textbuffer, "CONTROL_EXPOSURE failed");
						secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
						putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
						imshow("Status", statusimg);
					}

				} // end of if expchanged

			} // end of if ret == 1 block

		} // end of camera frames acquisition loop
		
		pCam->EndAcquisition();
		pCam->DeInit();
		pCam = nullptr;
		
		// Clear camera list before releasing system
    	camList.Clear();
    	
		// Release system
    	system->ReleaseInstance();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

    return result;
}
