/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/console/time.h>

namespace ecto {
  namespace pcl {
    struct EstimationBase
    {
      EstimationBase () {}

      virtual ~EstimationBase() {};

      static void declare_params(ecto::tendrils& params)
      {
        params.declare<int> ("k_search", "The number of k nearest neighbors to use for feature estimation.", 0);
        params.declare<double> ("radius_search", "The sphere radius to use for determining the nearest neighbors used for feature estimation.", 0);
        params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
        params.declare<std::string> ("name", "Feature name.", "feature");
        params.declare<bool> ("vervose", "Output information about the computation.", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<ecto::pcl::PointCloud> ("surface", "Cloud surface.");
        outputs.declare<ecto::pcl::FeatureCloud> ("output", "Cloud of features.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        k_ = params["k_search"];
        radius_ = params["radius_search"];
        locator_ = params["spatial_locator"];
        name_ = params["name"];
        vervose_ = params["vervose"];
        surface_ = inputs["surface"];
        output_ = outputs["output"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                          boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                          boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        throw ::pcl::IOException ("ERROR: EstimationBase is an interface class, it's not allow use it.");
      }

      ecto::spore<int> k_;
      ecto::spore<double> radius_;
      ecto::spore<int> locator_;
      ecto::spore<std::string> name_;
      ecto::spore<bool> vervose_;
      ecto::spore<ecto::pcl::PointCloud> surface_;
      ecto::spore<ecto::pcl::FeatureCloud> output_;
    };

    template<typename PointT, template <class A, class B, class C > class EstimatorT>
    struct Estimation : EstimationBase
    {
      Estimation() {}

      virtual ~Estimation() {}

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        EstimatorT<Point, ::pcl::Normal, PointT> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        impl.setKSearch(*k_);
        impl.setRadiusSearch(*radius_);
        typename ::pcl::search::KdTree<Point>::Ptr tree_ (new ::pcl::search::KdTree<Point>);
        impl.setSearchMethod(tree_);

        if (surface_->held)
          impl.setSearchSurface(surface_->cast< ::pcl::PointCloud<Point> >());
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

    };

    template<typename PointT, template <class A, class B, class C, class D> class EstimatorT>
    struct EstimationWithReferenceFrame : EstimationBase
    {
      EstimationWithReferenceFrame() {}

      virtual ~EstimationWithReferenceFrame() {}

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        EstimatorT<Point, ::pcl::Normal, PointT, ::pcl::ReferenceFrame> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        impl.setKSearch(*k_);
        impl.setRadiusSearch(*radius_);
        typename ::pcl::search::KdTree<Point>::Ptr tree_ (new ::pcl::search::KdTree<Point>);
        impl.setSearchMethod(tree_);

        if (surface_->held)
          impl.setSearchSurface(surface_->cast< ::pcl::PointCloud<Point> >());
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
    };

  }
}

