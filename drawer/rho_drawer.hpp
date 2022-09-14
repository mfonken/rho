//
//  rho_drawer.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 7/22/22.
//

#ifndef rho_drawer_hpp
#define rho_drawer_hpp

#include <stdio.h>

//#include "tau_structures.hpp"
#include "rho_structures.hpp"
#include "opencv_master.h"
//#include "tau_utility.hpp"

#define SIDEBAR_WIDTH 25
#define DENSITY_SCALE 1
//#define MAX_REGIONS 10

#define PROBABILITIES_FRAME_WIDTH 340
#define DETECTION_FRAME_WIDTH 500

#define INR(X,H)     INRLH(X,0,H)
#define INRLH(X,L,H) ((X>=H)?H-1:((X<L)?L:X))
#define OP_ALIGN(X,Y) (Y-X)

//#define DRAW_RHO_MAPS
#define RHO_MAPS_HEIGHT 200
#define RHO_MAPS_INSET 10

#define inseta_ 10
#define insetb_ 50
#define fontsize_ 1.4
#define fontcolora_ Vec3b(100,10,0)
#define fontcolorb_ Vec3b(0,100,10)
#define fontcolorc_ Vec3b(10,0,100)
#define frame_rate_display_delay 5

using namespace cv;

#define RDR_DEBUG

#ifdef RDR_DEBUG
#define LOG_RDR(L,...)          LOG(L,"<RhoDrawer> " __VA_ARGS__)
#define LOG_RDR_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#endif

class RhoDrawer
{
public:
    pthread_mutex_t drawer_mutex;
    cv::Mat image, frame,// RMX, RMY, //probabilities,
    rho_frame, rho_detection_x, rho_detection_y;
    int width, height, W, H, frame_i, x_detection_i, y_detection_i, pid_data_x, x_match_data_x, y_match_data_x, x_peak_data_x, y_peak_data_x;
    double *pid_data, *pid_data_target,
    *x_peak_data_l, *x_peak_data_u, *x_peak_data_uv, *x_peak_data_lv, *x_peak_data_ut, *x_peak_data_lt,
    *y_peak_data_l, *y_peak_data_u, *y_peak_data_uv, *y_peak_data_lv, *y_peak_data_ut, *y_peak_data_lt;
    int *x_match_data, *y_match_data;
    int scale_x = 0;
    
    double timestamp;
    int frame_rate_counter, frame_rate;
    
    rho_core_t * rho;
    
    RhoDrawer( rho_core_t * rho );
    virtual ~RhoDrawer();
    Mat DrawAll(void);
    void GetDensitiesFrame(Mat&);
    void DrawDensityGraph(Mat&);
    void DrawCurves(Mat&);
    void DrawRhoProbabilities(Mat&);
    Vec3b densityColor( int );
    int pixelDensity( Vec3b );
    Vec3b hsv2bgr(Vec3b);
    Mat& DrawRhoFrame(Mat &M);
    Mat& DrawRhoDetection(int dimension, Mat &M);
    void drawKalmans();
};


#endif /* rho_draweer_hpp */
