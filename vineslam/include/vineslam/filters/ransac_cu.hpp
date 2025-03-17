#pragma once
#include <chrono>
#include <iostream>
#include <vector>

#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Const.hpp>

namespace vineslam
{
// Declare ransac routine present on kernels/filters/ransac.cu
void ransac(float* xx, float* yy, float* zz, float* out_xx, float* out_yy, float* out_zz, int& size, int n_iters,
            float dist_threshold);

struct Ransac
{
  static void estimateNormal(const std::vector<Point>& points, float& a, float& b, float& c, float& d)
  {
    // -------------------------------------------------------------------------------
    // ----- Use PCA to refine th normal vector using all the inliers
    // -------------------------------------------------------------------------------
    // - 1st: assemble data matrix
    Eigen::MatrixXf data_mat;
    data_mat.conservativeResize(points.size(), 3);
    for (size_t i = 0; i < points.size(); i++)
    {
      Point pt = points[i];
      Eigen::Matrix<float, 1, 3> pt_mat(pt.x_, pt.y_, pt.z_);
      data_mat.block<1, 3>(i, 0) = pt_mat;
    }
    // - 2nd: calculate mean and subtract it to the data matrix
    Eigen::MatrixXf centered_mat = data_mat.rowwise() - data_mat.colwise().mean();
    // - 3rd: calculate covariance matrix
    Eigen::MatrixXf covariance_mat = (centered_mat.adjoint() * centered_mat);
    // - 4rd: calculate eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(covariance_mat);
    const Eigen::MatrixXf& eigen_vectors = eigen_solver.eigenvectors();

    Vec normal = Vec(eigen_vectors.col(0)[0], eigen_vectors.col(0)[1], eigen_vectors.col(0)[2]);

    // -------------------------------------------------------------------------------
    // ----- Normalize and save normal vector
    // -------------------------------------------------------------------------------
    normal.normalize();

    Point avg_pt(0, 0, 0);
    for (const auto& pt : points)
    {
      avg_pt.x_ += pt.x_;
      avg_pt.y_ += pt.y_;
      avg_pt.z_ += pt.z_;
    }
    avg_pt.x_ = (points.empty()) ? 0 : avg_pt.x_ / static_cast<float>(points.size());
    avg_pt.y_ = (points.empty()) ? 0 : avg_pt.y_ / static_cast<float>(points.size());
    avg_pt.z_ = (points.empty()) ? 0 : avg_pt.z_ / static_cast<float>(points.size());

    a = normal.x_;
    b = normal.y_;
    c = normal.z_;
    d = -(normal.x_ * avg_pt.x_ + normal.y_ * avg_pt.y_ + normal.z_ * avg_pt.z_);
  }

  static bool process(const std::vector<Point>& in_pts, Plane& out_plane, int max_iters = 20,
                      float dist_threshold = 0.08, bool filter_distant_pts = false)
  {
    // Filter distant points if necessary
    int ss = 0, s = in_pts.size();
    for (int i = 0; i < s; i++)
    {
      if ((in_pts[i].norm3D() < 5 && filter_distant_pts) || !filter_distant_pts)
      {
        ss++;
      }
    }
    if (ss == 0)
    {
      return false;
    }
    float *xx, *yy, *zz;
    float *out_xx, *out_yy, *out_zz;
    xx = (float*)malloc(ss * sizeof(float));
    yy = (float*)malloc(ss * sizeof(float));
    zz = (float*)malloc(ss * sizeof(float));
    out_xx = (float*)malloc(ss * sizeof(float));
    out_yy = (float*)malloc(ss * sizeof(float));
    out_zz = (float*)malloc(ss * sizeof(float));
    for (int i = 0, j = 0; i < in_pts.size(); i++)
    {
      if ((in_pts[i].norm3D() < 5 && filter_distant_pts) || !filter_distant_pts)
      {
        xx[j] = in_pts[i].x_;
        yy[j] = in_pts[i].y_;
        zz[j] = in_pts[i].z_;
        j++;
      }
    }

    // Call RANSAC GPU kernel
    ransac(xx, yy, zz, out_xx, out_yy, out_zz, ss, max_iters, dist_threshold);
    if (ss < 10)
    {
      free (xx);
      free (yy);
      free (zz);
      free (out_xx);
      free (out_yy);
      free (out_zz);
      return false;
    }

    // Parse result
    for (int i = 0; i < ss; i++)
    {
      vineslam::Point pt(out_xx[i], out_yy[i], out_zz[i]);
      out_plane.points_.push_back(pt);
    }

    // PCA-based normal refinement using all the inliers
    float l_a, l_b, l_c, l_d;
    estimateNormal(out_plane.points_, l_a, l_b, l_c, l_d);

    out_plane.a_ = l_a;
    out_plane.b_ = l_b;
    out_plane.c_ = l_c;
    out_plane.d_ = l_d;

    free (xx);
    free (yy);
    free (zz);
    free (out_xx);
    free (out_yy);
    free (out_zz);

    return true;
  }
};

}  // namespace vineslam