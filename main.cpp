#include <iostream>
#include "opencv2/opencv.hpp"
#include <chrono>
#include <signal.h>
#include "dxl.hpp"
#include <unistd.h>
#include <sys/time.h>

using namespace std;
using namespace cv;
using namespace chrono;

bool ctrl_c_pressed = false;
void ctrlc_handler(int){ ctrl_c_pressed = true; }

int main() {
	string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	VideoCapture source(src, CAP_GSTREAMER);
    if (!source.isOpened()){ cout << "Camera error" << endl; return -1; }

	Dxl mx;
   	int vel1 = 0,vel2 = 0;
	double ynum=0;

   	signal(SIGINT, ctrlc_handler);
   	if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }

	Mat frame, ROI, gray, correct, bin, morpology, labels, stats, centroids, colorimg;
	double error = 0, realline = 0;
	bool first = true, dxlmode=false;

	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
  	udpsink host=203.234.58.157 port=8001 sync=false";

    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.157 port=8002 sync=false";

    VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

	while (true) {
		auto start_time = high_resolution_clock::now();
		source >> frame;
		if (frame.empty()) { cout << "frame error" << endl; break; }

		writer1 << frame;

		ROI = frame(Rect(0, 270, 640, 90));
		cvtColor(ROI, gray, COLOR_BGR2GRAY);
		correct = gray + (Scalar(100) - mean(gray));

		adaptiveThreshold(correct, bin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 141, -21);
		morphologyEx(bin, morpology, MORPH_CLOSE, Mat(), Point(-1, -1), 6);

		int cnt = connectedComponentsWithStats(morpology, labels, stats, centroids);

		cvtColor(morpology, colorimg, COLOR_GRAY2BGR);

		double center_x = ROI.cols / 2.;
		double firstdist = 100, dist = 100;

		for (int i = 1; i < cnt; i++) {
			if (stats.at<int>(i, 4) < 200)continue;

			if (first) {
				if (abs(center_x - centroids.at<double>(i, 0)) < firstdist) {
					firstdist = abs(center_x - centroids.at<double>(i, 0));
				}
				if (abs(center_x - centroids.at<double>(i, 0)) == firstdist) {
					error = center_x - centroids.at<double>(i, 0);
					realline = centroids.at<double>(i, 0);

					rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
						stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(0, 0, 255), 2);
					circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
						1, Scalar(0, 0, 255), 2);
				}
				else {
					rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
						stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(255, 0, 0), 2);
					circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
						1, Scalar(255, 0, 0), 2);
				}
			}
			else {
				if (abs(realline - centroids.at<double>(i, 0)) < dist) {
					dist = abs(realline - centroids.at<double>(i, 0));
				}
				if (abs(realline - centroids.at<double>(i, 0)) == dist) {
					realline = centroids.at<double>(i, 0);
					ynum=centroids.at<double>(i,1);


					rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
						stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(0, 0, 255), 2);
					circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
						1, Scalar(0, 0, 255), 2);
				}
				else {
					rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
						stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(255, 0, 0), 2);
					circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
						1, Scalar(255, 0, 0), 2);
				}
			}
		}
		error = center_x-realline;
		if(error>265 || error<-265)circle(colorimg,Point(realline,ynum),1,Scalar(0,0,255),2);

		//100일때 0.15
		//200일때 0.3
		vel1 = 100 - 0.15* error;
       	vel2 = -(100 + 0.15* error);
		if(vel1<10) vel1=10;
		if(vel2>-10) vel2=-10;
		first = false;

		if(mx.kbhit()){
           	char c=mx.getch();
           	if(c=='s') dxlmode=true;
        }
        if (ctrl_c_pressed) break; 

		if(dxlmode){mx.setVelocity(vel1, vel2);}

        writer2 << colorimg;		

		//imshow("ROI", ROI);
		//imshow("correct", correct);
		//imshow("bin", bin);
		//imshow("morp", morpology);
		//imshow("color", colorimg);

		//waitKey(12);
		usleep(10*1000);
		auto end_time = high_resolution_clock::now();
		auto duration = duration_cast<milliseconds>(end_time - start_time);
		cout << "error: " << error <<", vel1: "<< vel1 << ", vel2: "<< vel2 
       		<<", time: "<< duration.count() << "ms" << endl;
		
	}
	mx.close();
	return 0;
}