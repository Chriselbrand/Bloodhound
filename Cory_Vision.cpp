#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include <wiringPi.h>
#include "/home/pi/wiringPi/wiringPi/wiringPi.h"
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;

int display = 0;
int Center_thresh, Canny_thresh, Vote_thresh, Min_thresh, Max_thresh; //these are for HoughCircles();

int Pins[7] = { 29, 28, 27, 26, 25, 24, 23 }; //40, 38, 36, 32, 37, 35, 33
int Transmit = 5; //18
int Receive = 6; //22
int Up = 22; //31
int Down = 21; //29
int Left = 4; //16
int Right = 3; //15

///Pin Segment:  TR,MM,BM,TM,BR,BL,TL
int One[7] = { 0, 1, 1, 1, 0, 1, 1 };
int Two[7] = { 0, 0, 0, 0, 1, 0, 1 };
int Three[7] = { 0, 0, 0, 0, 0, 1, 1 };
int Four[7] = { 0, 0, 1, 1, 0, 1, 0 };
int Five[7] = { 1, 0, 0, 0, 0, 1, 0 };
int Six[7] = { 1, 0, 0, 0, 0, 0, 0 };

Point center_cache; //center of cache pixel
int rad_cache = 0;  //radius of cache
Rect ROI;           //rectangle for ROI'ing
int cache_delta = 20;   //20 pixels off

//rectange components
int ROI_x = 0;      //top left corner.x
int ROI_y = 0;      //top left corner.y
int ROI_w = 0;      //width (x component)
int ROI_h = 0;      //height(y component)
int ROI_delta = 5;  //ROI extra

int grabber_x = 400;    //exact grabber location
int grabber_y = 145;    //exact grabber location
int delay_pulses = 110; //delay to send move pulses

int offsetx = 0; //either left or right offset
int offsety = 0; //either up or down offset

float pulsesx = 0;
float pulsesy = 0;
float pulse_per_inch = 20.6;

int left_pulses = 0;
int right_pulses = 0;
int up_pulses = 0;
int down_pulses = 0;

int done = 0;

void help(char** av)
{
    cout << "\nThis program justs gets you started reading images from video\n"
            "Usage:\n./"
         << av[0] << " <video device number>\n"
         << "q,Q,esc -- quit\n"
         << "space   -- save frame\n\n"
         << "\tThis is a starter sample, to get you up and going in a copy pasta fashion\n"
         << "\tThe program captures frames from a camera connected to your computer.\n"
         << "\tTo find the video device number, try ls /dev/video* \n"
         << "\tYou may also pass a video file, like my_vide.avi instead of a device number"
         << endl;
}
void showFrame(VideoCapture& c)
{
    if (display) {
        Mat frame;
        c >> frame;
        imshow("Debug", frame);
    }
}
int Process_Cache(VideoCapture& capture)
{
    Center_thresh = 555;
    Canny_thresh = 135;
    Vote_thresh = 15;
    Min_thresh = 110;
    Max_thresh = 120;

    cout << "Doing Cache Align" << endl;

    Mat Cache_mat, gray_cache;
    while (done != 1) {
        capture >> Cache_mat;

        if (Cache_mat.empty())
            break;

        cvtColor(Cache_mat, gray_cache, COLOR_BGR2GRAY);
        /// Reduce the noise so we avoid false circle detection; could add another GaussianBlur
        GaussianBlur(gray_cache, gray_cache, Size(5, 5), 2, 2);
        ///Create 3-param vector for cache(cache lid)
        vector<Vec3f> cache;
        //////////////Point center_cache(0, 0);         MOVED GLOBAL
        ///////////////////int rad_cache = 100;         MOVED GLOBAL

        /// Apply hough Transform to find Cache circle
        HoughCircles(gray_cache, cache, CV_HOUGH_GRADIENT, 1, Center_thresh, Canny_thresh, Vote_thresh, Min_thresh, Max_thresh);

        /// Draw Detected Pips
        for (int i = 0; i < cache.size(); i++) {
            center_cache = Point(cvRound(cache[i][0]), cvRound(cache[i][1]));
            rad_cache = cvRound(cache[i][2]);
            //cout << rad_cache << "   " << cache.size() << endl;
        }

        if (center_cache.x < grabber_x) { //(gray_cache.cols / 2)){         ///need to move LEFT
            offsetx = (grabber_x - center_cache.x); //(gray_cache.cols/2) - center_cache.x;  //number of pixels center_image away from center_cache
            pulsesx = ((float)offsetx / pulse_per_inch); //number of pulses to send to bot
            left_pulses = (int)round(pulsesx);
            int max_left = left_pulses;
            for (left_pulses; left_pulses >= 0; left_pulses--) {
                digitalWrite(Left, 1);
                delay(delay_pulses);
                digitalWrite(Left, 0);
                cout << "Left pulses" << left_pulses << endl;
                //showFrame(capture);
            } //cout<<max_left<<endl;
        }
        else if (center_cache.x > grabber_x) { //(gray_cache.cols / 2)){        ///need to move RIGHT
            offsetx = (center_cache.x - grabber_x); //(gray_cache.cols/2);  //number of pixels center_image away from center_cache
            pulsesx = ((float)offsetx / pulse_per_inch); //number of pulses to send to bot
            right_pulses = (int)round(pulsesx);
            int max_right = right_pulses;
            for (right_pulses; right_pulses >= 0; right_pulses--) {
                digitalWrite(Right, 1);
                delay(delay_pulses);
                digitalWrite(Right, 0);
                cout << "Right pulses" << right_pulses << endl;
                //showFrame(capture);
            } //cout<<max_right<<endl;
        }
        if (center_cache.y < grabber_y) { //((gray_cache.rows / 2)+50)){        ///need to move UP
            offsety = (grabber_y - center_cache.y); //(gray_cache.rows/2) - center_cache.y;  //number of pixels center_image away from center_cache
            pulsesy = ((float)offsety / pulse_per_inch); //number of pulses to send to bot
            up_pulses = (int)round(pulsesy);
            int max_up = up_pulses;
            for (up_pulses; up_pulses >= 0; up_pulses--) {
                digitalWrite(Up, 1);
                delay(delay_pulses);
                digitalWrite(Up, 0);
                cout << "Up pulses" << up_pulses << endl;
                //showFrame(capture);
            } //cout<<max_up<<endl;
        }
        if (center_cache.y > grabber_y) { //(gray_cache.rows / 2)){         ///need to move DOWN
            offsety = (center_cache.y - grabber_y); //(gray_cache.rows/2);  //number of pixels center_image away from center_cache
            pulsesy = ((float)offsety / pulse_per_inch); //number of pulses to send to bot
            down_pulses = (int)round(pulsesy);
            int max_down = down_pulses;
            for (down_pulses; down_pulses >= 0; down_pulses--) {
                digitalWrite(Down, 1);
                delay(delay_pulses);
                digitalWrite(Down, 0);
                cout << "Down pulses" << down_pulses << endl;
                //showFrame(capture);
            } //cout<<max_down<<endl;
        }
        if (center_cache.x <= grabber_x + cache_delta && center_cache.x >= grabber_x - cache_delta && center_cache.y <= grabber_y + cache_delta && center_cache.y >= grabber_y - cache_delta) {
            done = 1;
            cout << " done aligning" << endl;
        }
    }
    return 0;
}

int Process_Pips(VideoCapture& capture)
{
    Center_thresh = 5;
    Canny_thresh = 228;
    Vote_thresh = 15; 
    Min_thresh = 1;
    Max_thresh = 8;

    int Max_pips = 0;

    ROI_x = center_cache.x - (rad_cache + ROI_delta);
    ROI_y = center_cache.y - (rad_cache + ROI_delta);
    ROI_w = ROI_h = 2 * (rad_cache + ROI_delta);

    cout << "Doing Pips Process" << endl;
    Mat Pip_mat, gray_pips, gray_ROI;
    for (int i = 0; i < 10; i++) {

        capture >> Pip_mat;
        if (Pip_mat.empty())
            break;

        cvtColor(Pip_mat, gray_pips, COLOR_BGR2GRAY);
        /// Reduce the noise so we avoid false circle detection
        /// GaussianBlur( gray_pips, gray_pips, Size(9, 9), 2, 2 );
        gray_ROI = gray_pips(Rect(ROI_x, ROI_y, ROI_w, ROI_h));
        GaussianBlur(gray_ROI, gray_ROI, Size(5, 5), 2, 2);
        ///Create 3-param vector for circles(pips) and cache(cache lid)
        vector<Vec3f> circles;
        Point center_pips(0, 0);

        /// Apply hough Transform to find pip circles
        HoughCircles(gray_ROI, circles, CV_HOUGH_GRADIENT, 1, Center_thresh, Canny_thresh, Vote_thresh, Min_thresh, Max_thresh);

        /// Draw Detected Pips
        for (int i = 0; i < circles.size(); i++) {
            center_pips = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            cout << radius << "   " << circles.size() << endl;
        }
        ///Print number of Circles
        //cout << circles.size() << endl;

        ///Find Max number of pips
        //       int Max_pips = 0;
        if (Max_pips < circles.size()) {
            Max_pips = circles.size();
        }
    }
    ///Set 7-seg Display
    switch (Max_pips) {
    case 1:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], One[i]);
        }
        break;
    case 2:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Two[i]);
        }
        break;
    case 3:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Three[i]);
        }
        break;
    case 4:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Four[i]);
        }
        break;
    case 5:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Five[i]);
        }
        break;
    case 6:
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Six[i]);
        }
        break;
    default:
        cout << "No Pips Detected" << endl;
        for (int i = 0; i < 7; i++) {
            digitalWrite(Pins[i], Three[i]);
        }
        break;
    }
    return 0;
}

int main(int ac, char** av)
{
    wiringPiSetup();

    center_cache = Point(0,0);
    
    ///Set Transmit and Receive, U,D,L,R I/O
    pinMode(Transmit, OUTPUT);
    pinMode(Receive, INPUT);
    pinMode(Up, OUTPUT);
    pinMode(Down, OUTPUT);
    pinMode(Left, OUTPUT);
    pinMode(Right, OUTPUT);

    ///Set Transmit and U,D,L,R to LOW
    digitalWrite(Transmit, 0);
    digitalWrite(Up, 0);
    digitalWrite(Down, 0);
    digitalWrite(Left, 0);
    digitalWrite(Right, 0);

    ///Set 7-seg pins to output
    for (int i = 0; i < 7; i++) {
        pinMode(Pins[i], OUTPUT);
    }

    ///set 7-seg to 1 to turn off
    for (int i = 0; i < 7; i++) {
        digitalWrite(Pins[i], 1);
    }

    /*if (ac >= 2) {
        if (strcmp(av[1], "display") == 0) {
            cout << "Displaying frame in cache." << endl;
            display = 1;
        }
    }*/
    VideoCapture capture(0); //Initialize camera

    if (!capture.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }

    cout << "Waiting for Cache High. Receive: " << digitalRead(Receive) << endl;
    ///Wait until Kemp tells me to go
    while (digitalRead(Receive) != 1) {
        delay(10);
    }

    Process_Cache(capture);
    delay(100);

    digitalWrite(Transmit, 1); //tell bot to servo
    delay(100);
    digitalWrite(Transmit, 0);

    ///Wait until told to go
    cout << "Waiting for Pips High. Receive: " << digitalRead(Receive) << endl;
    while (digitalRead(Receive) != 1) {
        delay(10);
    }

    Process_Pips(capture);
    delay(100);

    digitalWrite(Transmit, 1); //tell bot I'm done
    delay(100);
    digitalWrite(Transmit, 0);

    return 0;
}
