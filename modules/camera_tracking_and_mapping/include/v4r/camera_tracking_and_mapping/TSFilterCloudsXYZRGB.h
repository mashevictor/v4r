/**
 * $Id$
 *
 * @author Johann Prankl
 *
 */

#ifndef KP_TSFILTER_CLOUDS_XYZRGB_H
#define KP_TSFILTER_CLOUDS_XYZRGB_H

#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <boost/shared_ptr.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/camera_tracking_and_mapping/Surfel.hh>
#include <list>
#include <v4r/keypoints/impl/triple.hpp>


#include <v4r/core/macros.h>


namespace v4r
{



/**
 * TSFilterCloudsXYZRGB
 */
class V4R_EXPORTS TSFilterCloudsXYZRGB
{
public:

  /**
   * Parameter
   */
  class Parameter
  {
  public:
    int batch_size_clouds;
    double cam_distance_select_frame; //[m]
    double angle_select_frame;        //[deg]
    float inv_depth_cut_off;
    int type;                         // interpolation/ mean value method: 0..original rgb, 1..mean value, (no interpolation), 2..bilin. interpolation, 3..bilin. inv_thr depth, bilin col
                                      // 4..bilin. inv_thr depth, keyframe col, 5..bilin. inv. depth, bilin. lookup rgb
    Parameter()
      : batch_size_clouds(15), cam_distance_select_frame(0.), angle_select_frame(0.), inv_depth_cut_off(0.01), type(3) {}
  };

 

private:
  Parameter param;

  double sqr_cam_distance_select_frame, cos_delta_angle_select_frame;
  int width, height;

  static std::vector<cv::Vec4i> npat;

  cv::Mat_<double> intrinsic, tgt_intrinsic;

  boost::mutex mtx_shm;
  double sf_timestamp;
  Eigen::Matrix4f sf_pose;
  v4r::DataMatrix2D<v4r::Surfel> sf_cloud;
  int sf_nb_frames;
  std::list< v4r::triple< pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::Matrix4f, double > > frames;

  pcl::PointCloud<pcl::PointXYZRGB> tmp_cloud;
  cv::Mat_<float> depth;
  cv::Mat_<float> depth_weight, col_weight;
  cv::Mat_<cv::Vec3f> im_bgr;
  cv::Mat_<cv::Vec3b> image, im_scaled;

  bool run, have_thread;

  boost::thread th_obectmanagement;
  boost::thread th_init;

  void operate();

  bool selectFrame(const Eigen::Matrix4f &pose0, const Eigen::Matrix4f &pose1);
  void integrateData(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void integrateDataRGB(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void integrateDataRGBbilinear(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void integrateDataRGBbilinear2(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void integrateDatabilinear(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void projectedColourTransfere(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose);
  void project3D(v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, const float &px_offs);
  void setColourValues(v4r::DataMatrix2D<Surfel> &_sf_cloud);
  void initKeyframe(const pcl::PointCloud<pcl::PointXYZRGB> &cloud0);

  inline float sqr(const float &d) {return d*d;}
  inline void insertWeightedXYZRGB(float &d, cv::Vec3f &col, float &w, const pcl::PointXYZRGB &pt, const float &w_pt);
  inline void insertWeightedXYZ(float &d, float &w, const pcl::PointXYZRGB &pt, const float &w_pt);
  inline void insertWeightedRGB(cv::Vec3f &col, float &w, const pcl::PointXYZRGB &pt, const float &w_pt);
  inline bool getInterpolatedRGB(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Point2f &pt, cv::Vec3f &rgb);
  inline void insertRGB(cv::Vec3f &col, float &w,  const cv::Vec3f &rgb);


public:
  cv::Mat dbg;

  TSFilterCloudsXYZRGB(const Parameter &p=Parameter());
  ~TSFilterCloudsXYZRGB();

  void start();
  void stop();

  inline bool isStarted() {return have_thread;}

  void reset();

  static void computeRadius(v4r::DataMatrix2D<Surfel> &sf_cloud, const cv::Mat_<double> &intrinsic);
  static void computeNormals(v4r::DataMatrix2D<Surfel> &sf_cloud, int nb_dist=1);

  void addCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose, const double &timestamp, bool have_track);

  inline void lock() { mtx_shm.lock(); }
  inline const double &getFiltTimestamp() { return sf_timestamp; }
  inline const Eigen::Matrix4f &getFiltPose() { return sf_pose; }
  inline const v4r::DataMatrix2D<v4r::Surfel> &getFiltCloud() { return sf_cloud; }
  inline void unlock() { mtx_shm.unlock(); }

  int getSurfelCloud(v4r::DataMatrix2D<Surfel> &cloud, Eigen::Matrix4f &pose, double &timestamp);
  int getFilteredCloudNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, Eigen::Matrix4f &pose, double &timestamp);
  int getFilteredCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Matrix4f &pose, double &timestamp);

  void setCameraParameterTSF(const cv::Mat &_intrinsic, int _width, int _height);
  void setCameraParameter(const cv::Mat &_intrinsic);
  void setParameter(const Parameter &p);

  typedef boost::shared_ptr< ::v4r::TSFilterCloudsXYZRGB> Ptr;
  typedef boost::shared_ptr< ::v4r::TSFilterCloudsXYZRGB const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline void TSFilterCloudsXYZRGB::insertWeightedXYZRGB(float &d, cv::Vec3f &col, float &w, const pcl::PointXYZRGB &pt, const float &w_pt)
{
  if (w>0)
  {
    if (fabs(1./d - 1./pt.z) < param.inv_depth_cut_off)
    {
      d = (w*d + w_pt*pt.z);
      col[0] = (w*col[0] + w_pt*pt.b);
      col[1] = (w*col[1] + w_pt*pt.g);
      col[2] = (w*col[2] + w_pt*pt.r);
      w += w_pt;
      d /= w;
      col[0] /= w;
      col[1] /= w;
      col[2] /= w;
    }
    else
    {
      w -= w_pt;
    }
  }
  else
  {
    d = pt.z;
    col[0] = pt.b;
    col[1] = pt.g;
    col[2] = pt.r;
    w=w_pt;
  }
}

inline void TSFilterCloudsXYZRGB::insertWeightedXYZ(float &d, float &w, const pcl::PointXYZRGB &pt, const float &w_pt)
{
  if (w>0)
  {
    if (fabs(1./d - 1./pt.z) < param.inv_depth_cut_off)
    {
      d = (w*d + w_pt*pt.z);
      w += w_pt;
      d /= w;

    }
    else
    {
      w -= w_pt;
    }
  }
  else
  {
    d = pt.z;
    w=w_pt;
  }
}

inline void TSFilterCloudsXYZRGB::insertWeightedRGB(cv::Vec3f &col, float &w, const pcl::PointXYZRGB &pt, const float &w_pt)
{
  col[0] = (w*col[0] + w_pt*pt.b);
  col[1] = (w*col[1] + w_pt*pt.g);
  col[2] = (w*col[2] + w_pt*pt.r);
  w += w_pt;
  col[0] /= w;
  col[1] /= w;
  col[2] /= w;
}

inline void TSFilterCloudsXYZRGB::insertRGB(cv::Vec3f &col, float &w, const cv::Vec3f &rgb)
{
  col[0] = (w*col[0] + rgb[2]);
  col[1] = (w*col[1] + rgb[1]);
  col[2] = (w*col[2] + rgb[0]);
  w += 1;
  col[0] /= w;
  col[1] /= w;
  col[2] /= w;
}


inline bool TSFilterCloudsXYZRGB::getInterpolatedRGB(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Point2f &pt, cv::Vec3f &rgb)
{
  if (pt.x>=0 && pt.y>=0 && pt.x<cloud.width-1 && pt.y<cloud.height-1)
  {
    int xt = (int) pt.x;
    int yt = (int) pt.y;
    float ax = pt.x - xt;
    float ay = pt.y - yt;
    rgb[0] = ( (1.-ax) * (1.-ay) * cloud(xt,yt).r +
                               ax     * (1.-ay) * cloud(xt+1,yt).r +
                              (1.-ax) *  ay     * cloud(xt,yt+1).r +
                               ax     *  ay     * cloud(xt+1,yt+1).r );
    rgb[1] = ( (1.-ax) * (1.-ay) * cloud(xt,yt).g +
                               ax     * (1.-ay) * cloud(xt+1,yt).g +
                              (1.-ax) *  ay     * cloud(xt,yt+1).g +
                               ax     *  ay     * cloud(xt+1,yt+1).g );
    rgb[2] = ( (1.-ax) * (1.-ay) * cloud(xt,yt).b +
                               ax     * (1.-ay) * cloud(xt+1,yt).b +
                              (1.-ax) *  ay     * cloud(xt,yt+1).b +
                               ax     *  ay     * cloud(xt+1,yt+1).b );
    return true;
  }
  else if(pt.x>=0 && pt.y>=0 && pt.x<cloud.width && pt.y<cloud.height )
  {
    const pcl::PointXYZRGB &pcl_pt = cloud((int)pt.x,(int)pt.y);
    rgb[0] = (float)pcl_pt.r;
    rgb[1] = (float)pcl_pt.g;
    rgb[2] = (float)pcl_pt.b;
    return true;
  }
  return false;
}

} //--END--

#endif

