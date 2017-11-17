#pragma once

// STL
#include <fstream>
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/texture_mapping.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Modified PCL functions
#include <v4r/surface_texturing/modifiedPclFunctions.h>

// Logging
#include <v4r/surface_texturing/Logger.h>

#include <v4r/core/macros.h>

// let's use tracking and mapping data structures :-)
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>

namespace v4r {

/*!
 * \brief The Coords struct     Coordinate class used in recursiveFindCoordinates for OdmTexturing::sortPatches().
 */
struct Coords {
  // Coordinates for row and column
  float r_, c_;

  // If coordinates have been placed
  bool success_;

  Coords() {
    r_ = 0.0;
    c_ = 0.0;
    success_ = false;
  }
};

/*!
 * \brief The Patch struct      Struct to hold all faces connected and with the same optimal camera.
 */
struct Patch {
  std::vector<size_t> faces_;
  float minu_, minv_, maxu_, maxv_;
  Coords c_;
  bool placed_;
  int materialIndex_;
  int optimalCameraIndex_;

  Patch() {
    placed_ = false;
    faces_ = std::vector<size_t>(0);
    minu_ = std::numeric_limits<double>::infinity();
    minv_ = std::numeric_limits<double>::infinity();
    maxu_ = 0.0;
    maxv_ = 0.0;
    optimalCameraIndex_ = -1;
    materialIndex_ = 0;
  }
};

/*!
 * \brief The Node struct       Node class for acceleration structure in OdmTexturing::sortPatches().
 */
struct Node {
  float r_, c_, width_, height_;
  bool used_;
  Node *rgt_;
  Node *lft_;

  Node() {
    r_ = 0.0;
    c_ = 0.0;
    width_ = 1.0;
    height_ = 1.0;
    used_ = false;
    rgt_ = nullptr;
    lft_ = nullptr;
  }

  Node(const Node &n) {
    r_ = n.r_;
    c_ = n.c_;
    used_ = n.used_;
    width_ = n.width_;
    height_ = n.height_;
    rgt_ = n.rgt_;
    lft_ = n.lft_;
  }
};

/*!
 * \brief   The OdmTexturing class is used to create textures to a welded ply-mesh using the camera
 *          positions from pmvs as input. The result is stored in an obj-file with corresponding
 *          mtl-file and the textures saved as jpg.
 */
class V4R_EXPORTS OdmTexturing {
 public:
  OdmTexturing(const float &delta_angle_view_normal = 20);
  ~OdmTexturing();

  /*!
   * \brief   run   Runs the texturing functionality using the provided input arguments.
   *                For a list of the accepted arguments, please see the main page documentation or
   *                call the program with parameter "-help".
   * \param   argc  Application argument count.
   * \param   argv  Argument values.
   * \return  0     if successful.
   */
  int run(int argc, char **argv);

  /**
   * @brief textureMapping
   * @param frames
   * @param mesh
   * @param dir
   * @param tex_mesh
   */
  void textureMapping(const std::vector<v4r::TSFFrame::Ptr> &frames, const pcl::PolygonMesh &mesh,
                      const cv::Mat_<double> &intrinsic, const Eigen::Matrix4f &base_transform,
                      const std::string &tmp_dir, const std::string &output_dir);

  /**
   * @brief setDeltaAngleViewNormal
   * @param delta_angle_view_normal
   */
  void setDeltaAngleViewNormal(const float &delta_angle_view_normal) {
    cosathr = cos(M_PI * delta_angle_view_normal / 180.);
  }

 private:
  /*!
   * \brief parseArguments    Parses command line arguments.
   * \param argc              Application argument count.
   * \param argv              Argument values.
   */
  void parseArguments(int argc, char **argv);

  /*!
   * \brief loadMesh          Loads a PLY-file containing vertices and faces.
   */
  void loadMesh();

  /*!
   * \brief loadCameras       Loads cameras from a bundle.out file with corresponding image list file.
   */
  void loadCameras();

  /*!
   * \brief triangleToImageAssignment     Assigns optimal camera to faces for the faces that are visible.
   */
  void triangleToImageAssignment();

  /*!
   * \brief calculatePatches      Arrange faces into patches as a prestep to arranging UV-mapping.
   */
  void calculatePatches();

  /*!
   * \brief recursiveFindCoords   Recursive function used in sortPatches() to find free area to place patch.
   * \param n                     The container in which to check for free space in.
   * \param w                     The width of the box to place.
   * \param h                     The height of the box to place.
   * \return                      The coordinates where the patch has been placed.
   */
  Coords recursiveFindCoords(Node &n, float w, float h);

  /*!
   * \brief sortPatches       Sorts patches into UV-containers to be used in createTextures() using a rectangle packer
   * approach.
   */
  void sortPatches();

  /*!
   * \brief createTextures    Creates textures to the mesh.
   */
  void createTextures();

  /*!
   * \brief writeObjFile      Writes the textured mesh to file on the OBJ format.
   */
  void writeObjFile();

  /*!
   * \brief printHelp         Prints help, explaining usage. Can be shown by calling the program with arguments:
   * "-help".
   */
  void printHelp();

  /**
   * @brief smoothNormalsClusterTriangles recursive clustering of triangles depending on the surface normal
   */
  void smoothNormalsClusterFaces(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<pcl::Vertices> &faces,
                                 std::vector<Eigen::Vector3f> &face_normals);
  int getNbFace(const std::vector<std::vector<int>> &face_indices, const std::vector<pcl::Vertices> &faces,
                int idx_face, int idx_vertex0, int idx_vertex1);
  void clusterFaces(const std::vector<std::vector<int>> &face_nbs, int idx,
                    const std::vector<Eigen::Vector3f> &face_normals, std::vector<int> &mask, Eigen::Vector3f &normal,
                    int &cnt, std::vector<int> &indices);
  void smoothNormalsTriangleNeighbours(const std::vector<std::vector<int>> &face_nbs,
                                       std::vector<Eigen::Vector3f> &face_normals);

  inline float getAbsCosaDeltaAngle(const Eigen::Vector3f &view_ray, const Eigen::Vector3f &normal);
  inline void addOrientedMeanNormal(const Eigen::Vector3f &normal, Eigen::Vector3f &mean_normal, int &cnt);

  int mcnt;
  float cosathr;

  Logger log_;              /**< Logging object. */
  std::string logFilePath_; /**< Path to store the log file. */

  std::string bundlePath_;     /**< Path to the bundle.out file. */
  std::string imagesPath_;     /**< Path to the folder with all images in the image list. */
  std::string imagesListPath_; /**< Path to the image list. */
  std::string inputModelPath_; /**< Path to the ply-file containing the mesh to be textured. */
  std::string outputFolder_;   /**< Path to the folder to store the output mesh and textures. */

  double bundleResizedTo_;   /**< The size used in the previous steps to calculate the camera focal_length. */
  double textureWithSize_;   /**< The desired size of the images to texture with. */
  double textureResolution_; /**< The resolution of each texture. */
  double padding_;           /**< A padding used to handle edge cases. */
  int nrTextures_;           /**< The number of textures created. */

  pcl::TextureMesh::Ptr mesh_;                 /**< PCL Texture Mesh */
  std::vector<Patch> patches_;                 /**< The vector containing all patches */
  pcl::texture_mapping::CameraVector cameras_; /**< The vector containing all cameras. */
  std::vector<int> tTIA_;                      /**< The vector containing the optimal cameras for all faces. */
};

class V4R_EXPORTS OdmTexturingException : public std::exception {
 public:
  OdmTexturingException() : message("Error in OdmTexturing") {}
  OdmTexturingException(std::string msgInit) : message("Error in OdmTexturing:\n" + msgInit) {}
  ~OdmTexturingException() throw() {}
  virtual const char *what() const throw() {
    return message.c_str();
  }

 private:
  std::string message; /**< The error message. */
};

/*************************** inline methods *****************************/
inline float OdmTexturing::getAbsCosaDeltaAngle(const Eigen::Vector3f &view_ray, const Eigen::Vector3f &normal) {
  return fabs(view_ray.dot(normal));
}

inline void OdmTexturing::addOrientedMeanNormal(const Eigen::Vector3f &normal, Eigen::Vector3f &mean_normal, int &cnt) {
  if (mean_normal.dot(normal) > 0)
    mean_normal = ((float)cnt) * mean_normal + normal;
  else
    mean_normal = ((float)cnt) * mean_normal + normal;

  cnt++;
  mean_normal.normalize();
}
}
