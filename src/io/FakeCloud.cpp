/*
 * FakeCloud.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: carlos
 */


#include <ecto_pcl/ecto_pcl.hpp>

namespace ecto {
  namespace pcl {
    struct FakeCloud
    {
      static void declare_params(ecto::tendrils& params)
      {
        params.declare<ecto::pcl::Format>("format", "Format of cloud found in PCD file.", ecto::pcl::FORMAT_XYZRGB);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud of features.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        format_ = params["format"];
        output_ = outputs["output"];
      }

      int process(const tendrils& inputs, const tendrils& outputs)
      {
        switch(*format_)
        {
          // Point Cloud cases
          case FORMAT_XYZ:
            {
              ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZ >);
              *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);
            } break;
          case FORMAT_XYZRGB:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
              *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);
            } break;
        }

        return ecto::OK;
      }

      spore<ecto::pcl::Format> format_;
      ecto::spore<ecto::pcl::PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::FakeCloud, "FakeCloud", "Generate a fake cloud, with no points.");
