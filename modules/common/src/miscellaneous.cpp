#include <v4r/common/miscellaneous.h>

#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

void transformNormals(const pcl::PointCloud<pcl::Normal> &normals_cloud, pcl::PointCloud<pcl::Normal> &normals_aligned,
                      const Eigen::Matrix4f &transform) {
  normals_aligned.points.resize(normals_cloud.points.size());
  normals_aligned.width = normals_cloud.width;
  normals_aligned.height = normals_cloud.height;

#pragma omp parallel for schedule(dynamic)
  for (size_t k = 0; k < normals_cloud.points.size(); k++) {
    Eigen::Vector3f nt(normals_cloud.points[k].normal_x, normals_cloud.points[k].normal_y,
                       normals_cloud.points[k].normal_z);
    normals_aligned.points[k].normal_x =
        static_cast<float>(transform(0, 0) * nt[0] + transform(0, 1) * nt[1] + transform(0, 2) * nt[2]);
    normals_aligned.points[k].normal_y =
        static_cast<float>(transform(1, 0) * nt[0] + transform(1, 1) * nt[1] + transform(1, 2) * nt[2]);
    normals_aligned.points[k].normal_z =
        static_cast<float>(transform(2, 0) * nt[0] + transform(2, 1) * nt[1] + transform(2, 2) * nt[2]);

    normals_aligned.points[k].curvature = normals_cloud.points[k].curvature;
  }
}

void transformNormals(const pcl::PointCloud<pcl::Normal> &normals_cloud, pcl::PointCloud<pcl::Normal> &normals_aligned,
                      const std::vector<int> &indices, const Eigen::Matrix4f &transform) {
  normals_aligned.points.resize(indices.size());
  normals_aligned.width = indices.size();
  normals_aligned.height = 1;
  for (size_t k = 0; k < indices.size(); k++) {
    Eigen::Vector3f nt(normals_cloud.points[indices[k]].normal_x, normals_cloud.points[indices[k]].normal_y,
                       normals_cloud.points[indices[k]].normal_z);

    normals_aligned.points[k].normal_x =
        static_cast<float>(transform(0, 0) * nt[0] + transform(0, 1) * nt[1] + transform(0, 2) * nt[2]);
    normals_aligned.points[k].normal_y =
        static_cast<float>(transform(1, 0) * nt[0] + transform(1, 1) * nt[1] + transform(1, 2) * nt[2]);
    normals_aligned.points[k].normal_z =
        static_cast<float>(transform(2, 0) * nt[0] + transform(2, 1) * nt[1] + transform(2, 2) * nt[2]);

    normals_aligned.points[k].curvature = normals_cloud.points[indices[k]].curvature;
  }
}

bool incrementVector(const std::vector<bool> &v, std::vector<bool> &inc_v) {
  inc_v = v;

  bool overflow = true;
  for (bool bit : v) {
    if (!bit) {
      overflow = false;
      break;
    }
  }

  bool carry = v.back();
  inc_v.back() = !v.back();
  for (int bit = v.size() - 2; bit >= 0; bit--) {
    inc_v[bit] = v[bit] != carry;
    carry = v[bit] && carry;
  }
  return overflow;
}

boost::dynamic_bitset<> computeMaskFromIndexMap(const Eigen::MatrixXi &image_map, size_t nr_points) {
  boost::dynamic_bitset<> mask(nr_points, 0);
  for (int i = 0; i < image_map.size(); i++) {
    int val = *(image_map.data() + i);
    if (val >= 0)
      mask.set(val);
  }

  return mask;
}

Eigen::Matrix3f computeRotationMatrixToAlignVectors(const Eigen::Vector3f &src, const Eigen::Vector3f &target) {
  Eigen::Vector3f A = src;
  Eigen::Vector3f B = target;

  A.normalize();
  B.normalize();

  float c = A.dot(B);

  if (c > 1.f - std::numeric_limits<float>::epsilon())
    return Eigen::Matrix3f::Identity();

  if (c < -1.f + std::numeric_limits<float>::epsilon()) {
    LOG(ERROR) << "Computing a rotation matrix of two opposite vectors is not supported by this equation. The returned "
                  "rotation matrix won't be a proper rotation matrix!";
    return -Eigen::Matrix3f::Identity();  // flip
  }

  const Eigen::Vector3f v = A.cross(B);

  Eigen::Matrix3f vx;
  vx << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

  return Eigen::Matrix3f::Identity() + vx + vx * vx / (1.f + c);
}

template <typename PointT>
V4R_EXPORTS void computePointCloudProperties(const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid,
                                             Eigen::Vector3f &elongationsXYZ, Eigen::Matrix4f &covariancePose,
                                             const std::vector<int> &indices) {
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
  EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;
  EIGEN_ALIGN16 Eigen::Matrix3f eigenBasis;

  if (indices.empty())
    computeMeanAndCovarianceMatrix(cloud, covariance_matrix, centroid);
  else
    computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix, centroid);

  pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);

  // create orthonormal rotation matrix from eigenvectors
  eigenBasis.col(0) = eigenVectors.col(0).normalized();
  float dotp12 = eigenVectors.col(1).dot(eigenBasis.col(0));
  Eigen::Vector3f eig2 = eigenVectors.col(1) - dotp12 * eigenBasis.col(0);
  eigenBasis.col(1) = eig2.normalized();
  Eigen::Vector3f eig3 = eigenBasis.col(0).cross(eigenBasis.col(1));
  eigenBasis.col(2) = eig3.normalized();

  // transform cluster into origin and align with eigenvectors
  Eigen::Matrix4f tf_rot = Eigen::Matrix4f::Identity();
  tf_rot.block<3, 3>(0, 0) = eigenBasis.transpose();
  Eigen::Matrix4f tf_trans = Eigen::Matrix4f::Identity();
  tf_trans.block<3, 1>(0, 3) = -centroid.topRows(3);

  covariancePose = tf_rot * tf_trans;
  //    Eigen::Matrix4f tf_rot = tf_rot_inv.inverse();

  // compute max elongations
  pcl::PointCloud<PointT> eigenvec_aligned;

  if (indices.empty())
    pcl::copyPointCloud(cloud, eigenvec_aligned);
  else
    pcl::copyPointCloud(cloud, indices, eigenvec_aligned);

  pcl::transformPointCloud(eigenvec_aligned, eigenvec_aligned, tf_rot * tf_trans);

  float xmin, ymin, xmax, ymax, zmin, zmax;
  xmin = ymin = xmax = ymax = zmin = zmax = 0.f;
  for (size_t pt = 0; pt < eigenvec_aligned.points.size(); pt++) {
    const PointT &p = eigenvec_aligned.points[pt];
    if (p.x < xmin)
      xmin = p.x;
    if (p.x > xmax)
      xmax = p.x;
    if (p.y < ymin)
      ymin = p.y;
    if (p.y > ymax)
      ymax = p.y;
    if (p.z < zmin)
      zmin = p.z;
    if (p.z > zmax)
      zmax = p.z;
  }

  elongationsXYZ(0) = xmax - xmin;
  elongationsXYZ(1) = ymax - ymin;
  elongationsXYZ(2) = zmax - zmin;
}

#define PCL_INSTANTIATE_computePointCloudProperties(T)                                                     \
  template V4R_EXPORTS void computePointCloudProperties<T>(const pcl::PointCloud<T> &, Eigen::Vector4f &,  \
                                                           Eigen::Vector3f &, Eigen::Matrix4f &eigenBasis, \
                                                           const std::vector<int> &);
PCL_INSTANTIATE(computePointCloudProperties, PCL_XYZ_POINT_TYPES)

template V4R_EXPORTS std::vector<size_t> createIndicesFromMask(const boost::dynamic_bitset<> &mask, bool invert);

template V4R_EXPORTS std::vector<int> createIndicesFromMask(const boost::dynamic_bitset<> &mask, bool invert);
}
