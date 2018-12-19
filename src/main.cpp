#include "ardrone/ardrone.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <math.h>

#define ERROR 		   -1
#define MAX_CLIENTS		2
#define MAX_DATA		1024

cv::Mat frame_;
int no_target;

void parent_read(int p[], int p2[]);
void child_write(int p[], int p2[]);

cv::Point center(cv::Rect r)
{
	cv::Point c(r.x + r.width/2,r.y + r.height/2);
	return c;
}
cv::Point center(cv::Mat f){
	cv::Point c(f.cols / 2,f.rows / 2);
	return c;
}

cv::Rect getTarget(cv::Mat frame){
	int maxid = -1;
	double maxsize = 0, area;
	std::vector<cv::Vec4i> hierarchy;
	std::vector< std::vector<cv::Point> > contours; 
	cv::Rect r1;
	findContours(frame, contours, hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
	for(int j=0; j < contours.size(); j++){
		area = fabs(contourArea(contours[j]));
		if(area > 100 && area > maxsize){
			maxid = j;
			maxsize = area;
		}
	}
	if(maxid == -1){
		r1.x = frame.cols / 2 - 50;
		r1.y = frame.rows / 2 - 50;
		r1.width = 100;
		r1.height = 100;
		no_target = 1;
		return r1;
	}
	no_target = 0;
	r1 = boundingRect(contours[maxid]);
	return r1;
}
void tracking(cv::Point c,cv::Point target,cv::Rect r){
	if(no_target == 1)
	{
		//lose tracking
		cv::putText(frame_, std::string("Lose tracking"), cv::Point(70,frame_.rows-70), 0, 2, cv::Scalar(250,250,250),2);
	}
	/*else
	{
		std::cout << "vy = " << c.x - target.x << " vz = " << c.y - target.y << std::endl;
	}*/

}


ARDrone ardrone;

int minH = 200, maxH = 255;
int minS = 110, maxS = 210;
int minV = 100, maxV = 210;
int frame_num = 0, track = 0, status_msg = 0,fly_count=0;
int frame_width, frame_height;
double integ_x[5] = {0}, integ_y[5] = {0}, integ_z[5] = {0};
cv::Mat hsv;
cv::Mat binalized;
cv::Ptr<cv::Tracker> tracker = cv::Tracker::create("MEDIANFLOW");
//cv::Ptr<cv::Tracker> tracker = cv::TrackerMedianFlow::create();
cv::Rect target;
cv::Rect2d bbox;
cv::Rect center_box;


std::string videoName="test0.avi";
cv::Size videoSize =cv::Size(640,360);
//cv::VideoWriter writer(videoName,CV_FOURCC('M','J','P','G'),15,videoSize);   

int first_send=0;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[]){
    // AR.Drone class
    //ARDrone ardrone;
    //ARDrone ardrone("192.168.0.100");

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }
    // Creating pipe between parent and child
    int p[2], p2[2];
	if (pipe(p) < 0)
		exit(1);
	if (pipe(p2) < 0)
		exit(-1);

	// Making the read end of the pipe non blocking
	if (fcntl(p[0], F_SETFL, O_NONBLOCK) < 0)
		exit(2);

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    /*int minH = 130, maxH = 160;
    int minS = 0, maxS = 255;
    int minV = 20, maxV = 255;
    int minH = 200, maxH = 255;
    int minS = 110, maxS = 210;
    int minV = 100, maxV = 210;
    int f_count = 0;
    double integ_x[5] = {0}, integ_y[5] = {0}, integ_z[5] = {0};*/
    //double sum_i_x = 0, sum_i_y = 0, sum_i_z = 0;
    //cv::Rect target, tmp;
    //cv::VideoCapture cap;
    
    //cv::Rect center_box;

    /*cv::Mat hsv;
    cv::Mat binalized;
	cv::Ptr<cv::Tracker> tracker = cv::Tracker::create("MEDIANFLOW");*/

    frame_ = ardrone.getImage();
    
    /*frame_width = frame_.cv::get(cv::CV_CAP_PROP_FRAME_WIDTH);
    frame_height = frame_.cv::get(cv::CV_CAP_PROP_FRAME_HEIGHT);
    video = cv::VideoWriter("outcpp.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(frame_width,frame_height));*/

    cv::cvtColor(frame_, hsv, cv::COLOR_BGR2HSV_FULL);

    // Binalize
    cv::Scalar lower(minH, minS, minV);
    cv::Scalar upper(maxH, maxS, maxV);
    cv::inRange(hsv, lower, upper, binalized);

    // De-noising
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);
    target = getTarget(binalized);

    //cv::Rect2d bbox;
    
    bbox.x = (double)target.x;
	bbox.y = (double)target.y;
	bbox.width = (double)target.width;
	bbox.height = (double)target.height;
    
	center_box.x = center(frame_).x-50;
	center_box.y = center(frame_).y-50;
	center_box.width = 100;
	center_box.height = 100;
    
    tracker->init(frame_.clone(),bbox);

    //			SAFE MODE
    while (1) 
    {
    
		int key = cv::waitKey(33);
	        if (key == 0x1b) break;

		frame_ = ardrone.getImage();
		//video.write(frame_);
		//writer.write(frame_);
		/*if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
            //break;
        }*/

		
		cv::cvtColor(frame_, hsv, cv::COLOR_BGR2HSV_FULL);

	    // Binalize
	    cv::Scalar lower(minH, minS, minV);
	    cv::Scalar upper(maxH, maxS, maxV);
	    cv::inRange(hsv, lower, upper, binalized);

	    // De-noising
	    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	    cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);
	    //imshow("binalized",binalized);

	    target = getTarget(binalized);

	    int x = tracker->update(frame_,bbox);

	    if(x == 0 && no_target == 0){
	        	bbox.x = (double)target.x;
				bbox.y = (double)target.y;
				bbox.width = (double)target.width;
				bbox.height = (double)target.height;
				tracker->clear();

				tracker = cv::Tracker::create("MEDIANFLOW");
				//tracker = cv::TrackerMedianFlow::create();
				tracker->init(frame_,bbox);
	    }
		cv::putText(frame_, std::string("safe mode"), cv::Point(70,70), 0, 2, cv::Scalar(0,255,0),2);
		cv::rectangle(frame_, target, cv::Scalar(0, 255, 0), 1);
		cv::rectangle(frame_, bbox, cv::Scalar( 255, 20, 100 ), 2);
		cv::rectangle(frame_, center_box, cv::Scalar(30,0,30), 2);
		cv::imshow("camera", frame_);
		//writer.write(frame_);
		if(norm(center(bbox) - center(frame_)) > 80){//frame_.cols/4->threshold		
			ardrone.takeoff();
			//cv::imwrite("frame.jpg", frame_);
			//sleep(1);
			track = 1;
			break;
		}
	
    }
    
    // 					TRACKING MODE
    pid_t pid = fork();

	// Creating child and parent processes
	switch(pid)
	{
		case -1:
			exit(3);

		case 0:	
			// child process
			child_write(p, p2);
			break;
		default:
			std::cout << "Parent process: " << getpid() << std::endl;
			parent_read(p, p2);
			break;
	}
    
    //sleep(3);
    
    return 0;
}

void parent_read(int p[], int p2[])
{
	int nread;
	char buf[MAX_DATA];
	char movStatus[] = "xxx";

	close(p[1]);		// Close the write end of the pipe for reading message from child process
	close(p2[0]);		// Close the read end of the pipe for writing message to child process

	static int64 last_t = cv::getTickCount();
	while (1)
    {
    	// Key input
    	int key = cv::waitKey(33);
    	if (key == 0x1b) break;

    	nread = read(p[0], buf, MAX_DATA);
    	switch(nread)
		{
			case -1:
				if (errno == EAGAIN)
				{
					break;
				}
			case 0:
				std::cout << "End of Connection" << std::endl;
				close(p[0]);
				close(p2[1]);
				ardrone.close();
				exit(8);
			default:
				buf[nread] = '\0';
				std::cout << nread << "\t" << getpid() << "\t" <<buf << std::endl;
				if (buf[0] == 's')
				{
					std::cout << nread << "\t" << getpid() << ":: MSG: " << buf << std::endl;
					//ardrone.takeoff();
				}
				else if (buf[0] == 't')
				{
					std::cout << nread << "\t" << getpid() << ":: status " << buf << "\ttrack : " << movStatus << std::endl;
					if (track)
					{
						status_msg = 1;
						cv::imwrite("frame.jpg", frame_);
			    		write(p2[1], movStatus, MAX_DATA);
			    		//status_msg = 0;
					}
				}
				else if (buf[0] == 'b')
				{
					std::cout << nread << "\t" << getpid() << ":: quitting " << buf << std::endl;
					close(p[0]);
					close(p2[1]);
					//ardrone.landing();
					key = 0x1b;
					//exit(9);
				}	
		}

        // Get an image
        //cv::Mat image = ardrone.getImage();
    	frame_ = ardrone.getImage();
    	//video.write(frame_);
    	//writer.write(frame_);
    	fly_count++;
		frame_num=fly_count%5;

    	//f_count=(f_count+1) % 300;

    	//std::cout<<"Tracking" << std::endl;
        cv::cvtColor(frame_, hsv, cv::COLOR_BGR2HSV_FULL);

        // Binalize
        cv::Scalar lower(minH, minS, minV);
        cv::Scalar upper(maxH, maxS, maxV);
        cv::inRange(hsv, lower, upper, binalized);

        // De-noising
        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);
        
        target = getTarget(binalized);
        int x = tracker->update(frame_,bbox);
    	if( x == 0 && no_target == 0 )
    	{
        	bbox.x = (double)target.x;
			bbox.y = (double)target.y;
			bbox.width = (double)target.width;
			bbox.height = (double)target.height;
			tracker->clear();
			tracker = cv::Tracker::create("MEDIANFLOW");
			//tracker = cv::TrackerMedianFlow::create();

			tracker->init(frame_,bbox);
			//tracker->update(frame_,bbox);
    	}
        cv::putText(frame_, std::string("Tracking mode"), cv::Point(70,70), 0, 2, cv::Scalar(0,255,0),2);
		cv::rectangle(frame_, target, cv::Scalar(0, 255, 0), 2);
		cv::rectangle(frame_,bbox, cv::Scalar( 255, 20, 100 ), 2);
		cv::rectangle(frame_,center_box, cv::Scalar(30,0,30),2);
		

		cv::Point c = center(frame_), t = center(bbox);

		tracking(c, t, target);

		
		// PID gains
		const double kp_x = 0.00001;
	    const double ki_x = 0.00005;
	    const double kd_x = 0.00001;	

	    const double kp_y = 0.001;
	    const double ki_y = 0.0001;
	    const double kd_y = 0.0001;

	    const double kp_z = 0.001;
	    const double ki_z = 0.0001;
	    const double kd_z = 0.0001;
	    // Errors
	    double error_x = center_box.area()-bbox.area(); //(c.y - t.y);   // Error front/back
	    double error_y = (c.x - t.x);   // Error left/right

	    
		double error_z = (c.y - t.y);
	    // Time [s]
	  
	    double dt = (cv::getTickCount() - last_t) / cv::getTickFrequency();
	    last_t = cv::getTickCount();

	    // Integral terms
	    static double integral_x = 0.0, integral_y = 0.0, integral_z=0.0;
	    if (dt > 0.1) { //
		    // Reset
	        integral_x = 0.0;
	        integral_y = 0.0;
	        integral_z = 0.0;
	    }
	    /*integral_x += error_x * dt;
	    integral_y += error_y * dt;
	    integral_z += error_z * dt;*/
	    //std::cout<< "dt::  " << dt <<std::endl;
	    
	    integral_x = integral_x + error_x * dt - integ_x[frame_num];
	    integral_y = integral_y + error_y * dt - integ_y[frame_num];
	    integral_z = integral_z + error_z * dt - integ_z[frame_num];


	    integ_x[frame_num] = error_x * dt;
	    integ_y[frame_num] = error_y * dt;
	    integ_z[frame_num] = error_z * dt;

	    


	    // Derivative terms
	    static double previous_error_x = 0.0, previous_error_y = 0.0, previous_error_z=0.0;
	    if (dt > 0.1) { //
		    // Reset
	        previous_error_x = 0.0;
	        previous_error_y = 0.0;
	        previous_error_z = 0.0;
	    }
	    double derivative_x = (error_x - previous_error_x) / dt;
	    double derivative_y = (error_y - previous_error_y) / dt;
	    double derivative_z = (error_z - previous_error_z) / dt;
	    previous_error_x = error_x;
	    previous_error_y = error_y;
	    previous_error_z = error_z;
	    
	    // Command velocities
	    double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	    vx = (kp_x * error_x) + (ki_x * integral_x) + (kd_x * derivative_x);; //kp * error_x + ki * integral_x + kd * derivative_x;
	    vy = 0.0;//(kp_y * error_y) + (ki_y * integral_y) + (kd_y * derivative_y);
	    vz = (kp_z * error_z) + (ki_z * integral_z) + (kd_z * derivative_z);//0.0;
	    //vr = 0.0;
	    vr = (kp_y * error_y) + (ki_y * integral_y) + (kd_y * derivative_y);
	    if (vx > 0.0)
			movStatus[0] = 'f';
		else
			movStatus[0] = 'b';

		if (vr > 0.0)
			movStatus[1] = 'l';
		else
			movStatus[1] = 'r';

		if (vz > 0.0)
			movStatus[2] = 'u';
		else
			movStatus[2] = 'd';
/*	    std::cout<<"e_y:"<<error_y<<std::endl;
	   	std::cout<<"i_y:"<<integral_y<<std::endl;
	    std::cout<<"d_y:"<<derivative_y<<std::endl;


	    std::cout << "(vx, vy, vz, vr)" << "(" << vx<<","<<vy << "," << vz <<","<< vr << ")" << std::endl << std::endl;
	    
	    if(no_target==1)
	    		ardrone.move3D(0,0,0,0.3);
	    else ardrone.move3D(vx, vy, vz, vr);
*/
	    if(no_target == 1)
	    	 {
	    	 	movStatus[0] = 'y';
	    	 	movStatus[1] = 'y';
	    	 	movStatus[2] = 'y';
	    	 	ardrone.move3D(0,0,0,0.3);
	    	 }
	    else ardrone.move3D(vx, vy, vz, vr);

		
	    /*
		// PID gains
		const double kp_x = 0.00001;
	    const double ki_x = 0.00005;
	    const double kd_x = 0.00001;	

	    const double kp_y = 0.001;
	    const double ki_y = 0.0001;
	    const double kd_y = 0.0001;

	    const double kp_z = 0.001;
	    const double ki_z = 0.0001;
	    const double kd_z = 0.0001;
	    
	    // Errors
	    double error_x = center_box.area() - bbox.area(); //(c.y - t.y);   // Error front/back
	    //double error_x = center_box.area() / bbox.area();
	    //error_x = error_x - 1;
	    double error_y = (c.x - t.x);   // Error left/right
		double error_z = (c.y - t.y);

		std::cout<<"center area :"<< center_box.area() << "\t bbox  "<< bbox.area() << std::endl;

		if (error_x > 0.0)
			movStatus[0] = 'f';
		else
			movStatus[0] = 'b';

		if (error_y > 0.0)
			movStatus[1] = 'l';
		else
			movStatus[1] = 'r';

		if (error_z > 0.0)
			movStatus[2] = 'u';
		else
			movStatus[2] = 'd';
   
	    // Time [s]
		double dt = (cv::getTickCount() - last_t) / cv::getTickFrequency();
	    last_t = cv::getTickCount();

	    // Integral terms
	    static double integral_x = 0.0, integral_y = 0.0, integral_z=0.0;
	    if (dt > 0.1) 
	    { //
		    // Reset
	        integral_x = 0.0;
	        integral_y = 0.0;
	        integral_z = 0.0;
	    }
	    integral_x += error_x * dt;
	    integral_y += error_y * dt;
	    integral_z += error_z * dt;
	    //std::cout<< "dt::  " << dt <<std::endl;
	    
	    integral_x = integral_x + error_x * dt - integ_x[frame_num];
	    integral_y = integral_y + error_y * dt - integ_y[frame_num];
	    integral_z = integral_z + error_z * dt - integ_z[frame_num];


	    integ_x[frame_num] = error_x * dt;
	    integ_y[frame_num] = error_y * dt;
	    integ_z[frame_num] = error_z * dt;

	    // Derivative terms
	    static double previous_error_x = 0.0, previous_error_y = 0.0, previous_error_z=0.0;
	    if (dt > 0.1) { //
		    // Reset
	        previous_error_x = 0.0;
	        previous_error_y = 0.0;
	        previous_error_z = 0.0;
	    }
	    double derivative_x = (error_x - previous_error_x) / dt;
	    double derivative_y = (error_y - previous_error_y) / dt;
	    double derivative_z = (error_z - previous_error_z) / dt;
	    previous_error_x = error_x;
	    previous_error_y = error_y;
	    previous_error_z = error_z;
	    
	    // Command velocities
	    double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	    vx = (kp_x * error_x) + (ki_x * integral_x) + (kd_x * derivative_x);; //kp * error_x + ki * integral_x + kd * derivative_x;
	    vy = 0;//(kp_y * error_y) + (ki_y * integral_y) + (kd_y * derivative_y);
	    vz = (kp_z * error_z) + (ki_z * integral_z) + (kd_z * derivative_z);//0.0;
	    //vr = 0.0;
	    vr = (kp_y * error_y) + (ki_y * integral_y) + (kd_y * derivative_y);

	    std::cout<<"e_x:"<<error_x<<std::endl;
	   	std::cout<<"i_x:"<<integral_x<<std::endl;
	    std::cout<<"d_x:"<<derivative_x<<std::endl;
	    std::cout << "(vx, vy, vz, vr)" << "(" << vx<<","<<vy << "," << vz <<","<< vr << ")" << std::endl << std::endl;
	    
	    if(no_target == 1)
	    	 {
	    	 	movStatus[0] = 'y';
	    	 	movStatus[1] = 'y';
	    	 	movStatus[2] = 'y';
	    	 	ardrone.move3D(0,0,0,0.3);
	    	 }
	    else ardrone.move3D(vx, vy, vz, vr);

        // Take off / Landing 
        /*if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }*/

        // Move
        vx = 0.0;
        vy = 0.0; 
        vz = 0.0; 
        vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        //std::cout << "(vy, vz)" << "(" << vx<<","<<vy << "," << vz <<","<< vr << ")" << std::endl;
        //ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        //cv::imshow("camera", image);
        cv::imshow("camera", frame_);
        if(fly_count == 100){
			cv::imwrite("frame.jpg", frame_);
			write(p2[1], "yes", MAX_DATA);
		}
        //writer.write(frame_);
        /*if(f_count == 0){
    		//cv::imshow("Image return",frame_);
    		//return image to user
    		cv::imwrite("frame.jpg", frame_);
    		//sleep(2);
    	}
    	*/
    }

    // See you
    ardrone.close();
    //writer.release();
}

void child_write(int p[], int p2[])
{
	int i, nread;
	close(p[0]);		// Close the read end of the pipe for sending message to the parent process
	close(p2[1]);		// Close the write end of the pipe for reading message from the parent process
	
	std::cout<< "child pid: " << getpid() << std::endl;

	struct sockaddr_in server_child, client_child;
	int sock_child, New, data_len;
	unsigned int sockaddr_len = sizeof(sockaddr_in);
	char data[MAX_DATA];

	//if ((sock_child = socket(AF_INET, SOCK_STREAM, 0)) == ERROR)
	if ((sock_child = socket(AF_INET, SOCK_DGRAM, 0)) == ERROR)
	{
		perror("server socket: ");
		exit(4);
	}
	server_child.sin_family = AF_INET;
	server_child.sin_port = htons(10000);
	server_child.sin_addr.s_addr = INADDR_ANY;
	bzero(&server_child.sin_zero, 8);

	if ((bind(sock_child, (struct sockaddr *)&server_child, sockaddr_len)) == ERROR)
	{
		perror("bind: ");
		exit(5);
	}
	/*if (listen(sock_child, MAX_CLIENTS) == ERROR)
	{
		perror("listen: ");
		exit(6);
	}*/

	std::cout << "Calling fbChat....." << std::endl;
	data_len = read(p2[0], data, MAX_DATA);

	// Creating child and parent processes
	switch(fork())
	{
		case -1:
			exit(7);

		case 0:	
			// child process
			system("python3 fbChat.py");
			break;
		default:
			std::cout << "server started..." << getpid() << std::endl;
			while (1)
			{
				//if ((New = accept(sock_child, (struct sockaddr *)&client_child, &sockaddr_len)) == ERROR)
				if ((data_len = recvfrom(sock_child, data, MAX_DATA,0, (struct sockaddr *)&client_child, &sockaddr_len)) == ERROR)
				{
					perror("accept: ");
					exit(-1);
				}

			std::cout << "New Client connected from port no. "<< ntohs(client_child.sin_port) << " and IP " 
				<< inet_ntoa(client_child.sin_addr) << std::endl;

			
				data[data_len] = '\0';
				std::cout << "Received message: " << data[0] << std::endl;
				if (data[0] == 's')
				{
					std::cout << "Starting: " << data << std::endl;
					write(p[1], data, MAX_DATA);
				}
				else if (data[0] == 't')
				{
					std::cout  << "Track: " << data << std::endl;
					write(p[1], data, MAX_DATA);
					data_len = read(p2[0], data, MAX_DATA);
					data[data_len] = '\0';
					std::cout  << "From Parent: " << data << std::endl;
					//if (data[0] == 'y')
						sendto(sock_child, data, MAX_DATA, 0, (struct sockaddr *)&client_child, sockaddr_len);
					//else
						//sendto(sock_child, "no", MAX_DATA, 0, (struct sockaddr *)&client_child, sockaddr_len);

				}
				else if (data[0] == 'b')
				{
					std::cout  << "Bye: " << data << std::endl;
					write(p[1], data, MAX_DATA);
					//data_len = -1;
					std::cout << "Client disconnected..." << std::endl;
					close(p[1]);
					close(p2[0]);
					close(sock_child);
					break;
				}
				//close(New);
				
			}
	}
	std::cout << "Drone Server Closed...." << std::endl;
}