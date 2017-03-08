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
    struct VFHEstimationImpl :  EstimationFromNormals<PointT, EstimatorT>
    {
      using EstimationBase::output_;
      using EstimationFromNormals<PointT, EstimatorT>::init;
      using EstimationFromNormals<PointT, EstimatorT>::compute;

      static void declare_params(ecto::tendrils& params)
      {
        EstimationFromNormals<PointT, EstimatorT>::declare_params(params);
        params.declare<float> ("vp_x", "Viewpoint x component.", 0.f);
        params.declare<float> ("vp_y", "Viewpoint y component.", 0.f);
        params.declare<float> ("vp_z", "Viewpoint z component.", 0.f);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        EstimationFromNormals<PointT, EstimatorT>::configure(params, inputs, outputs);
        vp_x_ = params["vp_x"], vp_y_ = params["vp_x"], vp_z_ = params["vp_x"];
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
        {
          *output_ = ecto::pcl::feature_cloud_variant_t(output);
          return code;
        }


        EstimationFromNormals<PointT, EstimatorT>::template compute<Point, PointT, EstimatorImplT > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }

      template<typename PointIn, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input,
                      const typename ::pcl::PointCloud< ::pcl::Normal >::ConstPtr& normals,
                      void* impl)
      {
        ReturnCode code = EstimationFromNormals<PointT, EstimatorT>::template init<PointIn, EstimatorImpl> (input, normals, impl);
        if(code != ecto::CONTINUE)
          return code;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        if (*vp_x_ != 0.0 || *vp_y_ != 0.0 || *vp_z_ != 0.0)
          impl_cast->setViewPoint(*vp_x_,*vp_y_,*vp_z_);
         else
           impl_cast->setViewPoint(input->sensor_origin_ [0],input->sensor_origin_ [1],input->sensor_origin_ [2]);

        return ecto::CONTINUE;
      }

      spore<float> vp_x_, vp_y_, vp_z_;
    };
  }
}



#endif /* SRC_FEATURES_VFHESTIMATION_HPP_ */
