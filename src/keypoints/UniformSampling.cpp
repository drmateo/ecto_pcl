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
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud of features.");
        outputs.declare<double> ("tictoc", "Process performance", std::numeric_limits<double>::max()).required(false);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        radius_ = params["radius"];
        output_ = outputs["output"];
        tictoc_ = outputs["tictoc"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::UniformSampling<Point> impl;
        typename ::pcl::PointCloud< Point >::Ptr keypoints(new typename ::pcl::PointCloud< Point >);
        keypoints->header = input->header;
        keypoints->sensor_origin_ = input->sensor_origin_;
        keypoints->sensor_orientation_ = input->sensor_orientation_;
        keypoints->width = 0;
        keypoints->height = 1;

        // If surface is declare but has less than 10 point stop process
        if (!input || input->size() == 0)
        {
          *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);
          return ecto::OK;
        }
        *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);

        // If input cluod has less than 10 point stop process
        if (input->size() < 10)
          return ecto::OK;

        impl.setRadiusSearch(*radius_);
        impl.setInputCloud(input);
        ::pcl::console::TicToc timer;
        timer.tic();
        impl.filter(*keypoints);
        double toc = timer.toc();
        LOG4CXX_INFO(logger_, "UniformSampling" << " took " << toc << "ms. for a cloud with " << input->size() << " points")

        *tictoc_ = toc;
        *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);
        return ecto::OK;
      }

      ecto::spore<float> radius_;
      ecto::spore<double> tictoc_;
      ecto::spore<ecto::pcl::PointCloud> output_;

      static log4cxx::LoggerPtr logger_ ;
    };

    log4cxx::LoggerPtr UniformSampling::logger_(log4cxx::Logger::getLogger("ecto.pcl"));
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::UniformSampling>, "UniformSampling", "Uniform Sampliing estimation");

