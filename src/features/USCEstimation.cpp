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

#include <pcl/features/usc.h>
#include "FeatureEstimator.hpp"

namespace ecto {
  namespace pcl {

    template <typename PointT = ::pcl::UniqueShapeContext1960, template <class A, class B, class C > class EstimatorT = ::pcl::UniqueShapeContext>
    struct USCEstimationImpl :  EstimationWithReferenceFrame<PointT, EstimatorT>
    {
      using EstimationBase::output_;
      using EstimationBase::init;
      using EstimationBase::compute;

      static void declare_params(ecto::tendrils& params)
      {
        EstimationWithReferenceFrame<PointT, EstimatorT>::declare_params(params);
        params.declare<float> ("minimal_radius", ".", 0.1f);
        params.declare<float> ("density_radius", ".", 0.1f);
        params.declare<float> ("rf_radius", ".", 2.f);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        EstimationWithReferenceFrame<PointT, EstimatorT>::configure(params, inputs, outputs);
        minimal_radius_ = params["minimal_radius"];
        density_radius_ = params["density_radius"];
        rf_radius_ = params["rf_radius"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        typedef EstimatorT<Point,  PointT, ::pcl::ReferenceFrame>  EstimatorImplT;
        EstimatorImplT impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        ReturnCode code = init<Point, EstimatorImplT > (input, reinterpret_cast<void*> (&impl));
        if(code != ecto::CONTINUE)
        {
          *output_ = ecto::pcl::feature_cloud_variant_t(output);
          return code;
        }

        EstimationBase::template compute<Point, PointT, EstimatorImplT > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }

      template<typename PointIn, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input,
                      void* impl)
      {
        ReturnCode code = EstimationBase::template init<PointIn, EstimatorImpl> (input, impl);
        if(code != ecto::CONTINUE)
          return code;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        impl_cast->setMinimalRadius(*minimal_radius_);
        impl_cast->setPointDensityRadius(*density_radius_);
        impl_cast->setLocalRadius(*rf_radius_);

        return ecto::CONTINUE;
      }

      spore<float> minimal_radius_;
      spore<float> density_radius_;
      spore<float> rf_radius_;
    };

    typedef ecto::pcl::USCEstimationImpl< > USCEstimation;
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::USCEstimation>,  "USCEstimation", "This cell provides Clustered Viewpoint Feature Histogram estimation.");

