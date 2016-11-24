#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int  get_pixel_value(float  x, float  y, Mat & image);
float get_pixel_value_gx(float  x, float  y, Mat *second_image_);
void ScaleofBilinear( Mat & src, Mat& dst, float scale);
void ScaleofNearest(const Mat & src, Mat& dst, float scale);

int main()
{
   Mat color = imread("/home/bobo/code/DirectmMethod_simple/data/1.png");
   if(color.empty())
     {
       cout<<"can not read the image"<<endl;
       return (-1);
     }
   Mat gray,out,out2;
   cvtColor(color,gray,cv::COLOR_BGR2GRAY);
   float scale = 1.6;
   /*
   cout<<color.step<<endl; //每一行的字节数
   cout<<color.step[0]<<endl; //每一行的字节数
   cout<<color.step[1]<<endl; //每一个矩阵元素的字节数 = elemSize
   */
   ScaleofNearest(gray,out,scale);
 ScaleofBilinear(gray,out2,scale);
 imshow("src",gray);
 imshow("nearest",out);
  imshow("bilinear",out2);
 cv::waitKey();
 return 0;
}

void ScaleofNearest(const Mat & src, Mat & dst, float scale)
{
  dst = Mat(Size(src.cols*scale,src.rows*scale), src.type(),Scalar::all(0));
  float scale_inv = 1./scale;
  for (int i=0; i < dst.rows ;i++)
    {
      int x=floor(i*scale_inv);
      x = min(x, src.rows-1);

      for (int j=0; j< dst.cols; j++)
        {
          int y= floor(j*scale_inv);
          y = min(y, src.cols-1);
          dst.at<uchar>(i,j) = src.at<uchar>(x,y);
        }
    }
}
void ScaleofBilinear( Mat & src, Mat& dst, float scale)
{
  dst = Mat(Size(src.cols*scale,src.rows*scale), src.type(),Scalar::all(0));
  float scale_inv = 1./scale;
  for (int i=0; i < dst.rows ;i++)
    {
      float x=i*scale_inv;
      for (int j=0; j< dst.cols; j++)
        {
          float y= j*scale_inv;
          dst.at<uchar>(i,j) = get_pixel_value_gx(x, y, &src);
        }
    }
}
//bilinear interpolation
//(x,y), (rows,cols),(行，列)
//x = sx + xx ,  sx为小于x的整数，xx为0-1的浮点数
//y = sy + yy， sy为小于x的整数，yy为0-1的浮点数
// f(x,y) =  (1-xx) * (1-yy)*src(sx,sy)
//                + xx * (1-yy)*src(sx+1,sy)
//                + (1-xx) * yy*src(sx,sy+1)
//                + xx * yy*src(sx+1,sy+1)
int get_pixel_value(float  x, float  y,  Mat &image)
{
  int sx = floor(x);//向下取整，取不大于x的整数，floor（9.99）=9
  int sy = floor(y);
  float xx =x - sx;
  float yy = y - sy;
 return int(
            (1-xx) * (1-yy) *image.at<uchar>(sx,sy)
            + xx*(1-yy)*image.at<uchar>(sx+1,sy)
            + (1-xx)*yy*image.at<uchar>(sx,sy+1)
            + xx*yy*image.at<uchar>(sx+1,sy+1)
            );

 //float value = second_image_.at<uchar>(x,y);
  //return value;
}

float get_pixel_value_gx(float  x, float  y, Mat *second_image_)
{
  uchar* data = & second_image_->data[int(x)*second_image_->step[0] + int (y) * second_image_->step[1]];
  float xx =x - floor(x);//向下取整，取不大于x的整数，floor（9.99）=9
  float yy = y - floor(y);
 return float(
             (1-xx) * (1-yy) *data[0]
             + xx*(1-yy)*data[second_image_->step]
             + (1-xx)*yy*data[1]
             + xx*yy*data[second_image_->step+1]
     );

 //float value = second_image_.at<uchar>(x,y);
  //return value;
}
