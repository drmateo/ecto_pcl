/*
 * ExtractIndicesFromLabels.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: carlos
 */




/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <iostream>
#include <vector>

namespace ecto
{
  namespace pcl
  {


    struct ExtractIndicesFromLabels
    {
      static void
      declare_params(tendrils& params) {}

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "The cloud to filter").required(true);
        outputs.declare<ecto::pcl::ClustersConstPtr>("clusters", "Clusters in the point cloud.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input_ = inputs["input"];
        clusters_ = outputs["clusters"];
      }

      int process(const tendrils& inputs, const tendrils& outputs)
      {
        const ::pcl::PointCloud< ::pcl::PointXYZRGBL>::ConstPtr& cloud = input_->cast< const ::pcl::PointCloud< ::pcl::PointXYZRGBL> >();

        // Initialize cluster list with at least one (for label 0)
        ecto::pcl::Clusters clusters;
        clusters.clear(), clusters.resize(1);
        clusters[0].header = cloud->header;

        // Wall throw the cloud to resolve cluster
        for (int i = 0; i < cloud->points.size(); i++)
        {
          uint32_t label = cloud->points[i].label;
          // Append a new cluster if it's found a new label, also it's added the clusters
          // for the labels bt the last found and the new one
          if (label >= clusters.size())
          {
            ecto::pcl::Indices indices ;
            indices.header = cloud->header;
            clusters.insert(clusters.end(), std::abs<int>(clusters.size() - label) + 1, indices);
          }

          clusters[label].indices.push_back(i);
        }

        // Expose clusters
        for (int i=0; i < clusters.size(); i++)
          clusters_->push_back(ecto::pcl::Indices::ConstPtr (new ecto::pcl::Indices(clusters[i])));

        return (ecto::OK);
      }

      ecto::spore<PointCloud> input_;
      ecto::spore<ecto::pcl::ClustersConstPtr> clusters_;
    };
  }
}// namespace ecto_acute

ECTO_CELL(ecto_pcl, ecto::pcl::ExtractIndicesFromLabels, "ExtractIndicesFromLabels", "Extract indices form a labeled clouds.")
