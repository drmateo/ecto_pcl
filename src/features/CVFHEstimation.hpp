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
      using EstimationFromNormals<PointT, EstimatorT>::compute;
      using VFHEstimationImpl<PointT, EstimatorT>::init;

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
        ReturnCode code = VFHEstimationImpl<PointT, EstimatorT>::template init<PointIn, EstimatorImpl> (input, normals, impl);
        if(code != ecto::CONTINUE)
          return code;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        impl_cast->setRadiusNormals(*radius_normals_);
        impl_cast->setClusterTolerance(*cluster_tolerance_);
        impl_cast->setEPSAngleThreshold(*angl_thr_);
        impl_cast->setCurvatureThreshold(*curv_thr_);
        impl_cast->setMinPoints(*cluster_size_);
        impl_cast->setNormalizeBins(*normalize_);
        // TODO
        impl_cast->setKSearch(0);

        return ecto::CONTINUE;
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
