/*
 * CVVHEstimation.hpp
 *
 *  Created on: Jan 20, 2017
 *      Author: carlos
 */

#ifndef SRC_FEATURES_CVFHESTIMATION_HPP_
#define SRC_FEATURES_CVFHESTIMATION_HPP_

#include "VFHEstimation.hpp"
#include <pcl/features/cvfh.h>

namespace ecto {
  namespace pcl {
    template <typename PointT = ::pcl::VFHSignature308, template <class A, class B, class C > class EstimatorT = ::pcl::CVFHEstimation>
    struct CVFHEstimationImpl :  VFHEstimationImpl<PointT, EstimatorT>
    {
      using Estimation<PointT, EstimatorT>::k_;
      using Estimation<PointT, EstimatorT>::radius_;
      using Estimation<PointT, EstimatorT>::locator_;
      using Estimation<PointT, EstimatorT>::name_;
      using Estimation<PointT, EstimatorT>::verbose_;
      using Estimation<PointT, EstimatorT>::tictoc_;
      using Estimation<PointT, EstimatorT>::surface_;
      using Estimation<PointT, EstimatorT>::output_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_x_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_y_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_z_;

      static void declare_params(ecto::tendrils& params)
      {
        VFHEstimationImpl<PointT, EstimatorT>::declare_params(params);
        params.declare<float> ("radius_normals", "The radius used to compute normals.", 0.005f * 3); // 3 times the leaf size
        params.declare<float> ("cluster_tolerance", "The maxim euclidean distance bt points to be added to the cluster.", 0.005f * 3); // 3 times the leaf size
        params.declare<float> ("angl_thr", "The deviation of the normlas between two points to be considered the same cluster.", 0.125f); // in rad.
        params.declare<float> ("curv_thr", "The curvature threshold to removing normals.", 0.03f);
        params.declare<int> ("cluster_size", "The minimum amount of points for a cluster to be considered.", 50);
        params.declare<bool> ("normalize", "Sets wether if the CVFH signatures should be normalized or not.", false);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        VFHEstimationImpl<PointT, EstimatorT>::configure(params, inputs, outputs);
        radius_normals_ = params["radius_normals"];
        cluster_tolerance_ = params["cluster_tolerance"];
        angl_thr_ = params["angl_thr"];
        curv_thr_ = params["curv_thr"];
        cluster_size_ = params["cluster_size"];
        normalize_ = params["normalize"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        EstimatorT<Point, ::pcl::Normal, PointT> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);
        ecto::spore<ecto::pcl::PointCloud>& surface = surface_;

        output->header = input->header;
        output->sensor_origin_ = input->sensor_origin_;
        output->sensor_orientation_ = input->sensor_orientation_;
        output->width = 0;
        output->height = 1;
        *output_ = ecto::pcl::feature_cloud_variant_t(output);

        // If surface is declare but has less than 10 point stop process
        if (surface->cast< typename ::pcl::PointCloud<Point> >()->size() < 10)
          return ecto::OK;

        // If there are not surface and input cluod has less than 10 point stop process
        if (!surface->held && input->size() < 10)
          return ecto::OK;

        if (*vp_x_ != 0.0 || *vp_y_ != 0.0 || *vp_z_ != 0.0)
             impl.setViewPoint(*vp_x_,*vp_y_,*vp_z_);
           else
             impl.setViewPoint(input->sensor_origin_ [0],input->sensor_origin_ [1],input->sensor_origin_ [2]);

        impl.setRadiusNormals(*radius_normals_);
        impl.setClusterTolerance(*cluster_tolerance_);
        impl.setEPSAngleThreshold(*angl_thr_);
        impl.setCurvatureThreshold(*curv_thr_);
        impl.setMinPoints(*cluster_size_);
        impl.setNormalizeBins(*normalize_);

        impl.setKSearch(*k_);
        impl.setRadiusSearch(*radius_);
        typename ::pcl::search::KdTree<Point>::Ptr tree_ (new ::pcl::search::KdTree<Point>);
        impl.setSearchMethod(tree_);

        if (surface->held)
          impl.setSearchSurface(surface->cast< typename ::pcl::PointCloud<Point> >());

        impl.setInputNormals(normals);
        impl.setInputCloud(input);

        ::pcl::console::TicToc timer;
        timer.tic();
        impl.compute(*output);
        if (*verbose_)
        {
          double toc = timer.toc();
          std::cout << *name_ << " took " << toc << "ms. for a cloud with " << input->size() << " points" << std::endl;
          *tictoc_ = toc;
        }

        *output_ = ecto::pcl::feature_cloud_variant_t(output);
        return ecto::OK;
      }

      spore<float> radius_normals_;
      spore<float> cluster_tolerance_;
      spore<float> angl_thr_;
      spore<float> curv_thr_;
      spore<int> cluster_size_;
      spore<bool> normalize_;
    };
  }
}



#endif /* SRC_FEATURES_CVFHESTIMATION_HPP_ */
