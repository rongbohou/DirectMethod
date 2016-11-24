
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/se3_ops.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sba/types_sba.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<opencv2/opencv.hpp>

namespace g2o {
//                                                                                        template_name<error's dimension,error's type,type of node>
class EdgeProjectDirect : public  BaseUnaryEdge<1, double, VertexSE3Expmap>
{
public: //method
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 EdgeProjectDirect()
  {}

  EdgeProjectDirect (Eigen::Vector3d point_3d, float fx, float fy, float u0, float v0 ,cv::Mat image)
  {
    xyz_world_ = point_3d;
    fx_ = fx;
    fy_ = fy;
    u0_ = u0;
    v0_= v0;
    second_image_ = image;
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

   virtual  void computeError();

//calculate the jocobin matrix
  virtual void linearizeOplus();
// get the pixel value of image(x,y)
protected:
  inline float get_pixel_value(float x, float y);

public: //variables member

  //3d point in the world frame
  Eigen::Vector3d xyz_world_;

  //camera intrinsics
  float fx_ = 0;
  float fy_ = 0;
  float u0_ = 0;
  float v0_= 0;

//second image
  cv::Mat  second_image_ ;//= nullptr;//?
};
} // end namespace
