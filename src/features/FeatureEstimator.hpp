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
#include <ecto_pcl/pcl_cell.hpp>
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
        params.declare<bool> ("verbose", "Output information about the computation.", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<ecto::pcl::PointCloud> ("surface", "Cloud surface.");
        outputs.declare<ecto::pcl::FeatureCloud> ("output", "Cloud of features.");
        outputs.declare<double> ("tictoc", "Process performance", std::numeric_limits<double>::max());
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        k_ = params["k_search"];
        radius_ = params["radius_search"];
        locator_ = params["spatial_locator"];
        name_ = params["name"];
        verbose_ = params["verbose"];
        surface_ = inputs["surface"];
        output_ = outputs["output"];
        tictoc_ = outputs["tictoc"];
      }

      template<typename Point, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<Point>::ConstPtr& input, void* impl)
      {
        ecto::spore<ecto::pcl::PointCloud>& surface = surface_;
        // If surface is declare but has less than 10 point stop process
        if (surface->cast< typename ::pcl::PointCloud<Point> >()->size() < 10)
          return ecto::OK;

        // If there are not surface and input cluod has less than 10 point stop process
        if (!surface_->held && input->size() < 10)
          return ecto::OK;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        if (*k_ != 0)
          impl_cast->setKSearch(*k_);
        if (*radius_ != 0.0)
          impl_cast->setRadiusSearch(*radius_);
        typename ::pcl::search::KdTree<Point>::Ptr tree_ (new ::pcl::search::KdTree<Point>);
        impl_cast->setSearchMethod(tree_);
        if (surface->held)
          impl_cast->setSearchSurface(surface->cast< ::pcl::PointCloud<Point> >());
        impl_cast->setInputCloud(input);

        return ecto::CONTINUE;
      }

      template<typename PointIn, typename PointOut, typename EstimatorImpl>
      void compute(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input, typename ::pcl::PointCloud<PointOut>::Ptr& output, void* impl)
      {
        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);

        output->header = input->header;
        output->sensor_origin_ = input->sensor_origin_;
        output->sensor_orientation_ = input->sensor_orientation_;
        output->width = 0;
        output->height = 1;
        *output_ = ecto::pcl::feature_cloud_variant_t(output);

        ::pcl::console::TicToc timer;
         timer.tic();
         impl_cast->compute(*output);
         if (*verbose_)
         {
           double toc = timer.toc();
           std::cout << *name_ << " took " << toc << "ms. for a cloud with " << input->size() << " points" << std::endl;
           *tictoc_ = toc;
         }

         *output_ = ecto::pcl::feature_cloud_variant_t(output);
      }

      ecto::spore<int> k_;
      ecto::spore<double> radius_;
      ecto::spore<int> locator_;
      ecto::spore<std::string> name_;
      ecto::spore<bool> verbose_;
      ecto::spore<double> tictoc_;
      ecto::spore<ecto::pcl::PointCloud> surface_;
      ecto::spore<ecto::pcl::FeatureCloud> output_;
    };

    template<typename PointT, template <class A, class B> class EstimatorT>
    struct Estimation : EstimationBase
    {
      Estimation() {}

      virtual ~Estimation() {}

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        EstimatorT<Point, PointT> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        ReturnCode code = init<Point, EstimatorT<Point, PointT> > (input, reinterpret_cast<void*> (&impl));
        if(code != ecto::CONTINUE)
          return code;

        compute<Point, PointT, EstimatorT<Point, PointT> > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }

    };

    template<typename PointT, template <class A, class B, class C > class EstimatorT>
    struct EstimationFromNormals : EstimationBase
    {
      EstimationFromNormals() {}

      virtual ~EstimationFromNormals() {}

      template<typename PointIn, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input,
                      const typename ::pcl::PointCloud< ::pcl::Normal >::ConstPtr& normals,
                      void* impl)
      {
        ReturnCode code = EstimationBase::init<PointIn, EstimatorImpl> (input, impl);
        if(code != ecto::CONTINUE)
          return code;

        // If there are not surface and input cluod has less than 10 point stop process
        if (normals->size() < 10)
          return ecto::OK;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        impl_cast->setInputNormals(normals);

        return ecto::CONTINUE;
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

        compute<Point, PointT, EstimatorImplT > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }

    };

    template<typename PointT, template <class A, class B, class C> class EstimatorT>
    struct EstimationWithReferenceFrame : EstimationBase
    {
      EstimationWithReferenceFrame() {}

      virtual ~EstimationWithReferenceFrame() {}

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        EstimatorT<Point, PointT, ::pcl::ReferenceFrame> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        ReturnCode code = init<Point, EstimatorT<Point, PointT, ::pcl::ReferenceFrame> > (input, reinterpret_cast<void*> (&impl));
        if(code != ecto::CONTINUE)
          return code;

        compute<Point, PointT, EstimatorT<Point, PointT, ::pcl::ReferenceFrame> > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }
    };

    template<typename PointT, template <class A, class B, class C, class D> class EstimatorT>
    struct EstimationFromNormalsWithReferenceFrame : EstimationBase
    {
      EstimationFromNormalsWithReferenceFrame() {}

      virtual ~EstimationFromNormalsWithReferenceFrame() {}

      template<typename PointIn, typename EstimatorImpl>
      ReturnCode init(const typename ::pcl::PointCloud<PointIn>::ConstPtr& input,
                      const typename ::pcl::PointCloud< ::pcl::Normal >::ConstPtr& normals,
                      void* impl)
      {
        ReturnCode code = EstimationBase::init<PointIn, EstimatorImpl> (input, impl);
        if(code != ecto::CONTINUE)
          return code;

        EstimatorImpl* impl_cast = reinterpret_cast<EstimatorImpl*> (impl);
        impl_cast->setInputNormals(normals);

        return ecto::CONTINUE;
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        EstimatorT<Point, ::pcl::Normal, PointT, ::pcl::ReferenceFrame> impl;
        typename ::pcl::PointCloud<PointT>::Ptr output(new typename ::pcl::PointCloud<PointT>);

        ReturnCode code = init<Point, EstimatorT<Point, ::pcl::Normal, PointT, ::pcl::ReferenceFrame> > (input, normals, reinterpret_cast<void*> (&impl));
        if(code != ecto::CONTINUE)
          return code;

        compute<Point, PointT, EstimatorT<Point, ::pcl::Normal, PointT, ::pcl::ReferenceFrame> > (input, output, reinterpret_cast<void*> (&impl));

        return ecto::OK;
      }
    };

  }
}

