/*
 * CVVHEstimation.hpp
 *
 *  Created on: Jan 20, 2017
 *      Author: carlos
 */

#ifndef SRC_FEATURES_VFHESTIMATION_HPP_
#define SRC_FEATURES_VFHESTIMATION_HPP_

#include "FeatureEstimator.hpp"
#include <pcl/features/vfh.h>

namespace ecto {
  namespace pcl {
    template <typename PointT = ::pcl::VFHSignature308, template <class A, class B, class C > class EstimatorT = ::pcl::VFHEstimation>
    struct VFHEstimationImpl :  Estimation<PointT, EstimatorT>
    {
      using Estimation<PointT, EstimatorT>::k_;
      using Estimation<PointT, EstimatorT>::radius_;
      using Estimation<PointT, EstimatorT>::locator_;
      using Estimation<PointT, EstimatorT>::name_;
      using Estimation<PointT, EstimatorT>::vervose_;
      using Estimation<PointT, EstimatorT>::surface_;
      using Estimation<PointT, EstimatorT>::output_;

      static void declare_params(ecto::tendrils& params)
      {
        Estimation<PointT, EstimatorT>::declare_params(params);
        params.declare<float> ("vp_x", "Viewpoint x component.", 0.f);
        params.declare<float> ("vp_y", "Viewpoint y component.", 0.f);
        params.declare<float> ("vp_z", "Viewpoint z component.", 0.f);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        Estimation<PointT, EstimatorT>::configure(params, inputs, outputs);
        vp_x_ = params["vp_x"], vp_y_ = params["vp_x"], vp_z_ = params["vp_x"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        EstimatorT<Point, ::pcl::Normal, PointT> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        if (*vp_x_ != 0.0 || *vp_y_ != 0.0 || *vp_z_ != 0.0)
             impl.setViewPoint(*vp_x_,*vp_y_,*vp_z_);
           else
             impl.setViewPoint(input->sensor_origin_ [0],input->sensor_origin_ [1],input->sensor_origin_ [2]);

        impl.setKSearch(*k_);
        impl.setRadiusSearch(*radius_);
        typename ::pcl::search::KdTree<Point>::Ptr tree_ (new ::pcl::search::KdTree<Point>);
        impl.setSearchMethod(tree_);

        ecto::spore<ecto::pcl::PointCloud>& surface = surface_;
        if (surface->held)
          impl.setSearchSurface(surface->cast< typename ::pcl::PointCloud<Point> >());
        impl.setInputNormals(normals);
        impl.setInputCloud(input);

        ::pcl::console::TicToc timer;
        timer.tic();
        impl.compute(*output);
        if (*vervose_)
          std::cout << *name_ << " took " << timer.toc() << "ms. for a cloud with " << input->size() << " points" << std::endl;

        output->header = input->header;
        output->sensor_origin_ = input->sensor_origin_;
        output->sensor_orientation_ = input->sensor_orientation_;
        *output_ = ecto::pcl::feature_cloud_variant_t(output);
        return ecto::OK;
      }

      spore<float> vp_x_, vp_y_, vp_z_;
    };
  }
}



#endif /* SRC_FEATURES_VFHESTIMATION_HPP_ */
