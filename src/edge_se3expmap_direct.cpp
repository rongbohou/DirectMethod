#include"edge_se3expmap_direct.h"
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include<Eigen/Dense>
//
namespace g2o {
  using namespace std;
  using namespace Eigen;

  void  EdgeProjectDirect::computeError()
  {
    const VertexSE3Expmap* Tcw = static_cast<const VertexSE3Expmap*>( _vertices[0] );
    Vector3d xyz_camera=Tcw->estimate().map(xyz_world_);
    float u = fx_*xyz_camera[0]/xyz_camera[2] + u0_;
    float v = fy_*xyz_camera[1]/xyz_camera[2] + v0_;
//check (x,y) in the image
    if ( (u-4)< 0 ||(u+4)> second_image_.cols ||(v-4)<0 ||(v+4) >second_image_.rows)
      {
        _error(0,0) = 0.0;
        this->setLevel(1);//?
      }
    else
      {
    double error = _measurement - get_pixel_value(u,v);
    _error(0,0)= error; //why _error(0,0)
      }
  }
//calculate the jocobian matrix
 void EdgeProjectDirect::linearizeOplus()
{
    if ( level() == 1)
      {
        _jacobianOplusXi = Matrix<double ,1 ,6>::Zero(); // ?
        return;

      }
    const VertexSE3Expmap* Tcw = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector3d xyz_camera=Tcw->estimate().map(xyz_world_);
    double x = xyz_camera[0];
    double y = xyz_camera[1];
    double z = xyz_camera[2];
    double z_2 = z * z;

    float u = fx_*x/z + u0_;
    float v = fy_*y/z + v0_;


    Matrix<double,  2,  6>  jacobian_uv_ksai;
    jacobian_uv_ksai(0,0) = x*y*fx_/z_2;
    jacobian_uv_ksai(0,1) = -(1+(x*x/z_2))*fx_;
    jacobian_uv_ksai(0,2) = fx_*y/z;
    jacobian_uv_ksai(0,3) = -1./z *fx_;
    jacobian_uv_ksai(0,4) = 0;
    jacobian_uv_ksai(0,5) = x/z_2*fx_;

    jacobian_uv_ksai(1,0) = (1+y*y/z_2)*fy_;
    jacobian_uv_ksai(1,1) = - x*y/z_2*fy_;
    jacobian_uv_ksai(1,2) = - x/z*fy_;
    jacobian_uv_ksai(1,3) = 0;
    jacobian_uv_ksai(1,4) =-1./z*fy_;
    jacobian_uv_ksai(1,5) =y/z_2*fy_;

    Matrix<double,  1,  2>  jacobian_pixelvalue_uv;//影响较大,如何更好地计算d(I(x,y))/d(x,y)
    //jacobian_pixelvalue_uv(0,0) = (get_pixel_value(u+1.0,v) - get_pixel_value(u-1,v))/2;//? /2
    //jacobian_pixelvalue_uv(0,1) = (get_pixel_value(u,v+1.0) - get_pixel_value(u,v-1))/2;
    jacobian_pixelvalue_uv(0,0) = (get_pixel_value(u+1.0,v) - get_pixel_value(u,v));//? /2
    jacobian_pixelvalue_uv(0,1) = (get_pixel_value(u,v+1.0) - get_pixel_value(u,v));
    _jacobianOplusXi = jacobian_pixelvalue_uv *jacobian_uv_ksai;

  }
  //x\y is float,bilinear interpolated
  inline float EdgeProjectDirect::get_pixel_value(float  x, float  y)
  {
    uchar* data = & second_image_.data[int(y)*second_image_.step + int (x)];
    float xx =x - floor(x);//向下取整，取不大于x的整数，floor（9.99）=9
    float yy = y - floor(y);
   return float(
                     (1-xx) * (1-yy) *data[0]
                  + xx*(1-yy)*data[1]
                  + (1-xx)*yy*data[second_image_.step]
                  + xx*yy*data[second_image_.step+1] );
  }
  //TODO
    bool EdgeProjectDirect::read(std::istream& is)
    {
      return true;
    }

    bool EdgeProjectDirect::write(std::ostream& os) const
    {
      return true;
    }
} //end namespace g2o
