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
      using EstimationFromNormals<PointT, EstimatorT>::compute;
      using CVFHEstimationImpl<PointT, EstimatorT>::init;

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
        typedef EstimatorT<Point, ::pcl::Normal, PointT>  EstimatorImplT;
        EstimatorImplT impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        ReturnCode code = init<Point, EstimatorImplT > (input, normals, reinterpret_cast<void*> (&impl));
        if(code != ecto::CONTINUE)
          return code;
        EstimationFromNormals<PointT, EstimatorT>::template compute<Point, PointT, EstimatorImplT > (input, output, reinterpret_cast<void*> (&impl));
        return ecto::OK;
      }

      template<typename PointIn, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input,
                      const typename ::pcl::PointCloud< ::pcl::Normal >::ConstPtr& normals,
                      void* impl)
      {
        ReturnCode code = CVFHEstimationImpl<PointT, EstimatorT>::template init<PointIn, EstimatorImpl> (input, normals, impl);
        if(code != ecto::CONTINUE)
          return code;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        impl_cast->setRefineClusters (*refine_clusters_);
        impl_cast->setAxisRatio(*axis_ratio_);
        impl_cast->setMinAxisValue(*min_axis_value_);

        return ecto::CONTINUE;
      }

      spore<float> refine_clusters_;
      spore<float> axis_ratio_;
      spore<float> min_axis_value_;
    };

    typedef OURCVFHEstimationImpl<> OURCVFHEstimation;
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCellWithNormals<ecto::pcl::OURCVFHEstimation>,  "OURCVFHEstimation", "This cell provides Oriented and Unique Clustered Viewpoint Feature Histogram estimation.");

