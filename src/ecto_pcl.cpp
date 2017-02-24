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

#include <boost/python.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

/* enumerations and values to be wrapped */
#include <pcl/sample_consensus/model_types.h>

namespace bp = boost::python;

#define ENUMSAC(r, data, ELEM)                              \
  .value(BOOST_PP_STRINGIZE(BOOST_PP_CAT(SACMODEL_, ELEM)), \
         BOOST_PP_CAT(pcl::SACMODEL_, ELEM))

#define MODELTYPES                              \
  (PLANE)                                       \
  (LINE)                                        \
  (CIRCLE2D)                                    \
  (CIRCLE3D)                                    \
  (SPHERE)                                      \
  (CYLINDER)                                    \
  (CONE)                                        \
  (TORUS)                                       \
  (PARALLEL_LINE)                               \
  (PERPENDICULAR_PLANE)                         \
  (PARALLEL_LINES)                              \
  (NORMAL_PLANE)                                \
  (REGISTRATION)                                \
  (PARALLEL_PLANE)                              \
  (NORMAL_PARALLEL_PLANE)

struct PointCloud2PointCloudT
{
  static void
  declare_params(tendrils& params)
  {
    params.declare<ecto::pcl::Format>("format", "Format of cloud to grab.", ecto::pcl::FORMAT_XYZRGB);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<ecto::pcl::PointCloud>("input", "An variant based PointCloud.");
    std::string doc = "An pcl::PointCloud<PointT> type.";
    switch (params.get<ecto::pcl::Format>("format"))
    {
      case ecto::pcl::FORMAT_XYZ:
    	  outputs.declare<pcl::PointCloud<pcl::PointXYZ>::ConstPtr >("output",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGB:
    	  outputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("output",doc );
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
    	  outputs.declare<pcl::PointCloud<pcl::PointNormal>::ConstPtr >("output",doc );
        break;
      case ecto::pcl::FORMAT_XYZI:
    	  outputs.declare<pcl::PointCloud<pcl::PointXYZI>::ConstPtr >("output",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
    	  outputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("output",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGBL:
    	  outputs.declare<pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr >("output",doc );
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    format_ = params["format"];
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int
  process(const tendrils& /*inputs*/, const tendrils& outputs)
  {
    switch (*format_)
    {
      case ecto::pcl::FORMAT_XYZ:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZ> >();
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZRGB> >();
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        output_ << input_->cast<pcl::PointCloud<pcl::PointNormal> >();
        break;
      case ecto::pcl::FORMAT_XYZI:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZI> >();
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZRGBA> >();
        break;
      case ecto::pcl::FORMAT_NORMAL:
        output_ << input_->cast<pcl::PointCloud<pcl::PointNormal> >();
        break;
      case ecto::pcl::FORMAT_XYZRGBL:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZRGBL> >();
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    return ecto::OK;
  }
  ecto::spore<ecto::pcl::Format> format_;
  ecto::spore<ecto::pcl::PointCloud> input_;
  ecto::tendril_ptr output_;
};

struct PointCloudT2PointCloud
{
  static void
  declare_params(tendrils& params)
  {
    params.declare<ecto::pcl::Format>("format", "Format of cloud to grab.", ecto::pcl::FORMAT_XYZRGB);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    std::string doc = "An pcl::PointCloud<PointT> type.";
    switch (params.get<ecto::pcl::Format>("format"))
    {
      case ecto::pcl::FORMAT_XYZ:
        inputs.declare<pcl::PointCloud<pcl::PointXYZ>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        inputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        inputs.declare<pcl::PointCloud<pcl::PointNormal>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZI:
        inputs.declare<pcl::PointCloud<pcl::PointXYZI>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        inputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGBL:
        inputs.declare<pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr >("input",doc );
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    outputs.declare<ecto::pcl::PointCloud>("output", "An variant based PointCloud.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    format_ = params["format"];
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int
  process(const tendrils& /*inputs*/, const tendrils& outputs)
  {
    switch (*format_)
    {
      case ecto::pcl::FORMAT_XYZ:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZ>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        *output_ = input_->get<pcl::PointCloud<pcl::PointNormal>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZI:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZI>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZRGBL:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr >();
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    return ecto::OK;
  }
  ecto::spore<ecto::pcl::Format> format_;
  ecto::spore<ecto::pcl::PointCloud> output_;
  ecto::tendril_ptr input_;
};

#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/stl_iterator.hpp>

#include<map>
#include<utility>

namespace bp=boost::python;

typedef std::map<int, std::vector<double> > Property;
typedef std::map<std::string, Property > Properties;

namespace ecto {
  namespace py {


    Properties dict_of_properties(bp::dict d)
    {
      Properties sl;

      bp::list keys = d.keys();
      bp::stl_input_iterator<std::string> key(keys),key_end;
      for (; key != key_end; ++key)
      {
        bp::list props =  bp::extract<bp::dict>(d[*key])().keys();
        bp::stl_input_iterator<int> prop(props),prop_end;
        for (; prop != prop_end; ++prop)
        {
          std::vector<double> value;
          bp::stl_input_iterator<double> begin((d[*key][*prop])),end;
          std::copy(begin,end,std::back_inserter(value));
          sl[  *key ][*prop] = value;
        }
      }
      return sl;
    }


  } // End of namespace py.
} // End of namespace ecto.

ECTO_DEFINE_MODULE(ecto_pcl)
{
  bp::enum_<pcl::SacModel>("SacModel")
    BOOST_PP_SEQ_FOR_EACH(ENUMSAC, ~, MODELTYPES)
    .export_values()
    ;

  bp::enum_<ecto::pcl::Format>("Format")
    .value("XYZ", ecto::pcl::FORMAT_XYZ)
    .value("XYZI", ecto::pcl::FORMAT_XYZI)
    .value("XYZRGB", ecto::pcl::FORMAT_XYZRGB)
    .value("XYZRGBA", ecto::pcl::FORMAT_XYZRGBA)
    .value("XYZRGBL",ecto::pcl::FORMAT_XYZRGBL)
    .value("XYZRGBNORMAL",ecto::pcl::FORMAT_XYZRGBNORMAL)
    .value("POINTNORMAL",ecto::pcl::FORMAT_POINTNORMAL)

    .value("NORMAL", ecto::pcl::FORMAT_NORMAL)
    .value("PFHSIGNATURE", ecto::pcl::FORMAT_PFHSIGNATURE)
    .value("FPFHSIGNATURE", ecto::pcl::FORMAT_FPFHSIGNATURE)
    .value("VFHSIGNATURE", ecto::pcl::FORMAT_VFHSIGNATURE)
    .value("SHOT", ecto::pcl::FORMAT_SHOT)
    .value("UNIQUESHAPECONTEXT", ecto::pcl::FORMAT_UNIQUESHAPECONTEXT)
    .value("SHAPECONTEXT", ecto::pcl::FORMAT_SHAPECONTEXT)
    .value("ESFSIGNATURE", ecto::pcl::FORMAT_ESFSIGNATURE)
    .value("GRSDSIGNATURE", ecto::pcl::FORMAT_GRSDSIGNATURE)
    .value("PRINCIPALRADIIRSD", ecto::pcl::FORMAT_PRINCIPALRADIIRSD)
    .value("HISTOGRAM153", ecto::pcl::FORMAT_HISTOGRAM153)
    .export_values()
    ;

  bp::scope().attr("KDTREE_FLANN") = 0;
  bp::scope().attr("KDTREE_ORGANIZED_INDEX") = 1;

  bp::class_<Properties > ("MapOfProperties")
    .def(bp::map_indexing_suite<Properties > ());

  bp::def("dict_of_properties", &ecto::py::dict_of_properties);

}


namespace ecto {
  namespace pcl {
    typedef PointCloud2PointCloudT PointCloud2PointCloudT;
    typedef PointCloudT2PointCloud PointCloudT2PointCloud;
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PointCloud2PointCloudT, "PointCloud2PointCloudT",
          "Convert a generic variant based PointCloud to a strongly typed pcl::PointCloud<pcl::PointT>.")
ECTO_CELL(ecto_pcl, ecto::pcl::PointCloudT2PointCloud, "PointCloudT2PointCloud",
          "Convert a strongly typed pcl::PointCloud<pcl::PointT> to a generic variant based PointCloud.")

