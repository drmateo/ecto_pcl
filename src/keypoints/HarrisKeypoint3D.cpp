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

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/console/time.h>

namespace ecto {
  namespace pcl {
    struct HarrisKeypoint3D
    {
      static void declare_params(ecto::tendrils& params)
      {
        params.declare<int> ("method", "The method to use: HARRIS(0), NOBLE(1), LOWE(2), TOMASI(3), CURVATURE(4).", 0);
        params.declare<float> ("threshold", "The threshold value for detecting corners.", 0.0f);
        params.declare<float> ("radius", "The radius for normal estimation and non maxima suppression.", 0.01f);
        params.declare<bool> ("non_max_suppression", "Whether non maxima suppression should be applied or the response for each point should be returned.", false);
        params.declare<bool> ("do_refine", "Whether the detected key points should be refined or not.", true);
        params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).", 0);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud of features.");
        outputs.declare<double> ("tictoc", "Process performance", std::numeric_limits<double>::max());
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        method_ = params["method"];
        threshold_ = params["threshold"];
        radius_ = params["radius"];
        non_max_suppression_ = params["non_max_suppression"];
        do_refine_ = params["do_refine"];
        locator_ = params["spatial_locator"];
        output_ = outputs["output"];
        tictoc_ = outputs["tictoc"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                  boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
      {
        ::pcl::HarrisKeypoint3D<Point, ::pcl::PointXYZI, ::pcl::Normal> impl;
        typename ::pcl::PointCloud< ::pcl::PointXYZI >::Ptr keypoints(new typename ::pcl::PointCloud< ::pcl::PointXYZI >);

        keypoints->header = input->header;
        keypoints->sensor_origin_ = input->sensor_origin_;
        keypoints->sensor_orientation_ = input->sensor_orientation_;
        keypoints->width = 0;
        keypoints->height = 1;
        *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);

        // If input cluod has less than 10 point stop process
        if (input->size() < 10)
          return ecto::OK;

        typename ::pcl::search::Search<Point>::Ptr tree_;
        switch (*locator_)
        {
          case 0:
          {
            tree_.reset (new ::pcl::search::KdTree<Point>);
            break;
          }
          case 1:
          {
            tree_.reset (new ::pcl::search::OrganizedNeighbor<Point>);
            break;
          }
        }
        impl.setSearchMethod(tree_);

        switch (*method_)
        {
          case 0:
          {
            impl.setMethod(impl.HARRIS);
            break;
          }
          case 1:
          {
            impl.setMethod(impl.NOBLE);
            break;
          }
          case 2:
          {
            impl.setMethod(impl.LOWE);
            break;
          }
          case 3:
          {
            impl.setMethod(impl.TOMASI);
            break;
          }
          case 4:
          {
            impl.setMethod(impl.CURVATURE);
            break;
          }
        }

        impl.setRadius(*radius_);
        impl.setThreshold(*threshold_);
        impl.setNonMaxSupression(*non_max_suppression_);
        impl.setRefine(*do_refine_);

        impl.setInputCloud(input);
        impl.setNormals(normals);

        ::pcl::console::TicToc timer;
        timer.tic();
        impl.compute(*keypoints);
        double toc = timer.toc();
        LOG4CXX_INFO(logger_, "HarrisKeypoint3D" << " took " << toc << "ms. for a cloud with " << input->size() << " points")

        *tictoc_ = toc;

        *output_ = ecto::pcl::xyz_cloud_variant_t(keypoints);
        return ecto::OK;
      }

      ecto::spore<int> method_;
      ecto::spore<float> threshold_, radius_;
      ecto::spore<bool> non_max_suppression_, do_refine_;
      ecto::spore<int> locator_;
      ecto::spore<double> tictoc_;
      ecto::spore<ecto::pcl::PointCloud> output_;

      static log4cxx::LoggerPtr logger_ ;
    };

    log4cxx::LoggerPtr HarrisKeypoint3D::logger_(log4cxx::Logger::getLogger("ecto.pcl"));
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCellWithNormals<ecto::pcl::HarrisKeypoint3D>, "HarrisKeypoint3D", "Harris keypoint 3D estimation");

