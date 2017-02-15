/*
 * UniformSampling.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: carlos
 */


#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/console/time.h>

namespace ecto {
  namespace pcl {
    struct UniformSampling
    {
      static void declare_params(ecto::tendrils& params)
      {
        params.declare<float> ("radius", "The radius for normal estimation and non maxima suppression.", 0.01f);
        params.declare<bool> ("verbose", "Output information about the computation.", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud of features.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        radius_ = params["radius"];
        verbose_ = params["verbose"];
        output_ = outputs["output"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::UniformSampling<Point> impl;
        typename ::pcl::PointCloud< Point >::Ptr keypoints(new typename ::pcl::PointCloud< Point >);

        impl.setRadiusSearch(*radius_);
        impl.setInputCloud(input);
        ::pcl::console::TicToc timer;
        timer.tic();
        impl.filter(*keypoints);
        if (*verbose_)
          std::cout << "UniformSampling" << " took " << timer.toc() << "ms. for a cloud with " << input->size() << " points" << std::endl;

        keypoints->header = input->header;
        keypoints->sensor_origin_ = input->sensor_origin_;
        keypoints->sensor_orientation_ = input->sensor_orientation_;
        *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);
        return ecto::OK;
      }

      ecto::spore<float> radius_;
      ecto::spore<bool> verbose_;
      ecto::spore<ecto::pcl::PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::UniformSampling>, "UniformSampling", "Uniform Sampliing estimation");

