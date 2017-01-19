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
#include <ecto_pcl/pcl_cell.hpp>
#include <pcl/io/io.h>
#include <string>

namespace ecto {
  namespace pcl {
    struct CopyCloud
    {
      static void declare_params(tendrils& params)
      {
        params.declare<Format> ("output_format", "Format of output cloud.", FORMAT_XYZ);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud> ("output", "Output cloud.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        format_ = params["output_format"];
        output_ = outputs["output"];
      }
      
      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs, 
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        switch(*format_)
        {
          case FORMAT_XYZ:
          {
            typename ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud(new typename ::pcl::PointCloud< ::pcl::PointXYZ >);
            ::pcl::copyPointCloud<Point, ::pcl::PointXYZ> (*input, *cloud);
            *output_ = xyz_cloud_variant_t(cloud);
            break;
          }

          case FORMAT_XYZRGBL:
          {
            typename ::pcl::PointCloud< ::pcl::PointXYZRGBL >::Ptr cloud(new typename ::pcl::PointCloud< ::pcl::PointXYZRGBL >);
            ::pcl::copyPointCloud<Point, ::pcl::PointXYZRGBL> (*input, *cloud);
            *output_ = xyz_cloud_variant_t(cloud);
            break;
          }
        }
        return ecto::OK;
      }
      
      spore<PointCloud> output_;
      spore<Format> format_;
    };
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::CopyCloud>, "CopyCloud", "Copy the input cloud on the output cloud, they cloud be of diff types.");

