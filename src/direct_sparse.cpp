#include<iostream>
#include <vector>
#include<stdlib.h>

#include"edge_se3expmap_direct.h"
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<g2o/core/robust_kernel.h>
#include<g2o/types/sba/types_six_dof_expmap.h>

#include<opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include<Eigen/Dense>

using namespace std;
using namespace Eigen;
// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    float cx, cy, fx, fy, scale;
};
//(x,y), (cols,rows)
Eigen::Vector3d imageTo3d(int x, int y, ushort&d, const CAMERA_INTRINSIC_PARAMETERS &camera)
{
   float z_c = float(d)/camera.scale;
   float x_c = z_c*(x - camera.cx)/camera.fx;
   float y_c = z_c*(y - camera.cy)/camera.fy;
   return Eigen::Vector3d(x_c, y_c, z_c);
}

Eigen::Vector2d C_3dToimage(Vector3d point,const CAMERA_INTRINSIC_PARAMETERS &camera)
{
  float u = point[0]*camera.fx/point[2] + camera.cx;
  float v = point[1]*camera.fy/point[2] + camera.cy;
  return Vector2d(u,v);
}

//direct method to estimate the cemara pose
bool poseEstimationDirect(const vector<float>& measurements, const vector<Vector3d>& world_points, CAMERA_INTRINSIC_PARAMETERS& camera, cv::Mat& gray, Eigen::Isometry3d& Tcw);
int main()
{
    char color_path[256] , depth_path[256];
    int image_num = 2;
    cv::Mat color_image,  pre_color,depth_image, gray_image;
    string directSettingsFile = "/home/bobo/code/DirectmMethod_simple/config/kinect1.yaml";
    cv::FileStorage fsSettings(directSettingsFile, cv::FileStorage::READ);
           if(!fsSettings.isOpened())
           {
              cerr << "Failed to open settings file at: " <<directSettingsFile<< endl;
              exit(-1);
           }
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = fsSettings["Camera.cx"];
    camera.cy = fsSettings["Camera.cy"];
    camera.fx = fsSettings["Camera.fx"];
    camera.fy = fsSettings["Camera.fy"];
    camera.scale = fsSettings["DepthMapFactor"];

    vector<Eigen::Vector3d> world_points;
    vector<float> measurements;
    Isometry3d Tcw = Isometry3d::Identity();
     for(int index = 0; index < image_num; index++)
    {
         sprintf(color_path,"/home/bobo/code/DirectmMethod_simple/data/rgb_png/%01d.png",index);
         sprintf(depth_path,"/home/bobo/code/DirectmMethod_simple/data/depth_png/%01d.png",index);
         color_image = cv::imread(color_path);
         depth_image= cv::imread(depth_path);
         if(color_image.empty())
           {
             cout<<"can not read the image"<<endl;
             return (-1);
           }
         cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);

         if( index == 0)
           {
             cv::Mat imgShow ;
             vector<cv::KeyPoint> keypoints;
             cv::FastFeatureDetector fast;
             fast.detect(color_image, keypoints);
             cv::drawKeypoints(color_image, keypoints, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
             cv::imshow( "keypoints", imgShow );

             for(auto kp:keypoints)
               {
                 // 去掉邻近边缘处的点
                 if ( kp.pt.x < 20 || kp.pt.y < 20 || ( kp.pt.x+20 ) >color_image.cols || ( kp.pt.y+20 ) >color_image.rows )
                     continue;

                 ushort d = depth_image.ptr<ushort>(int(kp.pt.y))[int(kp.pt.x)];
                 if(d == 0)
                   continue;

                 Eigen::Vector3d p3d;
                 p3d = imageTo3d(kp.pt.x, kp.pt.y, d, camera);
                 float imagegray = float( gray_image.at<uchar>(kp.pt.y, kp.pt.x) );
                 world_points.push_back(p3d);
                 measurements.push_back(imagegray);
               }
           cout <<"size of points :"<<world_points.size()<<endl;
           cout<<"size of measurement: "<< measurements.size()<<endl;


           //cv::waitKey(0);
           color_image.copyTo(pre_color);
           continue;
           }
        poseEstimationDirect(measurements,world_points, camera, gray_image, Tcw);
        cout<<"Tcw"<<Tcw.matrix()<<endl;

        //plot the feature points
      cv::Mat img_show(color_image.rows*2,color_image.cols,color_image.type());
      pre_color.copyTo(img_show(cv::Rect(0,0,color_image.cols,color_image.rows)));
      color_image.copyTo(img_show(cv::Rect(0,color_image.rows,color_image.cols,color_image.rows)));
      for(int i=0; i < world_points.size(); i++)
        {
           if ( rand() > RAND_MAX/5 )
                continue;
          Vector3d p =world_points.at(i);
          Vector2d pixel_pre = C_3dToimage(p,camera);

         Vector3d p2 =Tcw *p;
        // Vector3d p2 =Tcw.rotation()*p + Tcw.translation();
         Vector2d pixel_current = C_3dToimage(p2,camera);
          if(pixel_current[0] <0||pixel_current[0]>color_image.cols ||pixel_current[1]<0||pixel_current[1] > color_image.rows)
            continue;
          int b = rand() % 255;
          int g = rand() % 255;
          int r = rand() % 255;
          cv::circle(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]), 8, cv::Scalar(b,g,r), 2);
          cv::circle(img_show,cv::Point2f(pixel_current[0],pixel_current[1]+color_image.rows), 8, cv::Scalar(b,g,r), 2);
          cv::line(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]),cv::Point2f(pixel_current[0],pixel_current[1]+color_image.rows),cv::Scalar(b,g,r),1);
        }
      cv::imshow("result", img_show);
      cv::waitKey();
    }

     return 0;

}
bool poseEstimationDirect(const vector<float>& measurements, const vector<Vector3d>& world_points, CAMERA_INTRINSIC_PARAMETERS& camera, cv::Mat& gray, Isometry3d& Tcw)
{
  //init g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock; //优化的向量为6X1
  DirectBlock::LinearSolverType * linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
  DirectBlock* solver_ptr = new DirectBlock(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);//输出调试信息

  g2o::VertexSE3Expmap* Vcw = new g2o::VertexSE3Expmap();
  //Vcw->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
  Vcw->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
  Vcw->setId( 0 );
 optimizer.addVertex(Vcw);

 //add edge
     for(int id = 0; id < world_points.size(); id ++ )
 {
   g2o::EdgeProjectDirect* edge = new g2o::EdgeProjectDirect(world_points.at(id), camera.fx, camera.fy, camera.cx, camera.cy, gray);
   edge->setVertex(0, Vcw);
   edge->setMeasurement(measurements.at(id));
   edge->setInformation(Matrix<double,1,1>::Identity());
   edge->setId(id+1);
   //cout<<id<<endl;
   optimizer.addEdge(edge);
 }

 cout<<"edge in the graph"<<optimizer.edges().size()<<endl;
 optimizer.initializeOptimization();
 optimizer.optimize(30);
 Tcw = Vcw->estimate();
}

