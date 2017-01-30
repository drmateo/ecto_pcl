/*
 * Copyright (c) 2017, Carlos M. Mateo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "CVFHEstimation.hpp"
#include <pcl/features/our_cvfh.h>

namespace ecto {
  namespace pcl {
    template <typename PointT = ::pcl::VFHSignature308, template <class A, class B, class C > class EstimatorT = ::pcl::OURCVFHEstimation>
    struct OURCVFHEstimationImpl :  CVFHEstimationImpl<PointT, EstimatorT>
    {
      using Estimation<PointT, EstimatorT>::k_;
      using Estimation<PointT, EstimatorT>::radius_;
      using Estimation<PointT, EstimatorT>::locator_;
      using Estimation<PointT, EstimatorT>::name_;
      using Estimation<PointT, EstimatorT>::vervose_;
      using Estimation<PointT, EstimatorT>::surface_;
      using Estimation<PointT, EstimatorT>::output_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_x_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_y_;
      using VFHEstimationImpl<PointT, EstimatorT>::vp_z_;
      using CVFHEstimationImpl<PointT, EstimatorT>::radius_normals_;
      using CVFHEstimationImpl<PointT, EstimatorT>::cluster_tolerance_;
      using CVFHEstimationImpl<PointT, EstimatorT>::angl_thr_;
      using CVFHEstimationImpl<PointT, EstimatorT>::curv_thr_;
      using CVFHEstimationImpl<PointT, EstimatorT>::cluster_size_;
      using CVFHEstimationImpl<PointT, EstimatorT>::normalize_;

      static void declare_params(ecto::tendrils& params)
      {
        CVFHEstimationImpl<PointT, EstimatorT>::declare_params(params);
        params.declare<float> ("refine_clusters", "The factor used to decide if a point is used to estimate a stable cluster.", 1.f);
        params.declare<float> ("axis_ratio", "Sets the min axis ratio between the SGURF axes to decide if disambiguation is feasible. ", 0.8f);
        params.declare<float> ("min_axis_value", "Sets the min disambiguation axis value to generate several SGURFs for the cluster when disambiguation is difficult.", 0.925f);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        CVFHEstimationImpl<PointT, EstimatorT>::configure(params, inputs, outputs);
        refine_clusters_ = params["refine_clusters"];
        axis_ratio_ = params["axis_ratio"];
        min_axis_value_ = params["min_axis_value"];
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

        impl.setRadiusNormals(*radius_normals_);
        impl.setClusterTolerance(*cluster_tolerance_);
        impl.setEPSAngleThreshold(*angl_thr_);
        impl.setCurvatureThreshold(*curv_thr_);
        impl.setMinPoints(*cluster_size_);
        impl.setNormalizeBins(*normalize_);

        impl.setRefineClusters (*refine_clusters_);
        impl.setAxisRatio(*axis_ratio_);
        impl.setMinAxisValue(*min_axis_value_);

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
        *output_ = ecto::pcl::feature_cloud_variant_t(output);
        return ecto::OK;
      }

      spore<float> refine_clusters_;
      spore<float> axis_ratio_;
      spore<float> min_axis_value_;
    };

    typedef OURCVFHEstimationImpl<> OURCVFHEstimation;
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCellWithNormals<ecto::pcl::OURCVFHEstimation>,  "OURCVFHEstimation", "This cell provides Oriented and Unique Clustered Viewpoint Feature Histogram estimation.");

