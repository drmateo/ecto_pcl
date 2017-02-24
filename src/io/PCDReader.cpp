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

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <pcl/io/pcd_io.h>
//#include <pcl/conversions.h>

namespace ecto {
  namespace pcl {

    template<typename PointT> int
    loadPCD(spore<std::string> file_buffer, typename ::pcl::PointCloud<PointT>::Ptr& cloud, spore<bool> buffer)
    {
      // Read PCD from buffer string
      if (*buffer)
      {
        std::istringstream buffer_ss (*file_buffer);
        return ::pcl::io::loadPCDBuffer< PointT > (buffer_ss, *cloud);
      }

      // Read PCD from file
      return ::pcl::io::loadPCDFile< PointT > (*file_buffer, *cloud);
    }

    struct PCDReader
    {
      static void declare_params(tendrils& params)
      {
        params.declare<ecto::pcl::Format>("format", "Format of cloud found in PCD file.",
                                           ecto::pcl::FORMAT_XYZRGB);
        params.declare<std::string> ("filename", "Name of the pcd file", "");
        params.declare<bool>("buffer",
                             "Define if the PCD has to be read from buffer string, in case that case filename is handle has a buffer",
                             false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        switch(params["format"]->get<ecto::pcl::Format>())
        {
          case FORMAT_XYZ:
          case FORMAT_XYZRGB:
          case FORMAT_XYZRGBL:
            outputs.declare<PointCloud>("output", "A point cloud from the pcd file.");
            break;
          case FORMAT_PFHSIGNATURE:
          case FORMAT_FPFHSIGNATURE:
          case FORMAT_VFHSIGNATURE:
          case FORMAT_SHOT:
          case FORMAT_UNIQUESHAPECONTEXT:
          case FORMAT_ESFSIGNATURE:
          case FORMAT_SHAPECONTEXT:
          case FORMAT_GRSDSIGNATURE:
          case FORMAT_PRINCIPALRADIIRSD:
          case FORMAT_HISTOGRAM153:
            outputs.declare<FeatureCloud>("output", "A point cloud from the pcd file.");
        }
      }

      PCDReader() {}
      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        format_ = params["format"];

        switch(*format_)
        {
          case FORMAT_XYZ:
          case FORMAT_XYZRGB:
          case FORMAT_XYZRGBL:
            cloud_ = outputs["output"];
            break;
          case FORMAT_PFHSIGNATURE:
          case FORMAT_FPFHSIGNATURE:
          case FORMAT_VFHSIGNATURE:
          case FORMAT_SHOT:
          case FORMAT_UNIQUESHAPECONTEXT:
          case FORMAT_ESFSIGNATURE:
          case FORMAT_SHAPECONTEXT:
          case FORMAT_GRSDSIGNATURE:
          case FORMAT_PRINCIPALRADIIRSD:
          case FORMAT_HISTOGRAM153:
            feature_ = outputs["output"];
        }

        filename_ = params["filename"];

        buffer_ = params["buffer"];
      }

      int process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        switch(*format_)
        {
          // Point Cloud cases
          case FORMAT_XYZ:
            {
              std::cout << "opening " << *filename_ << std::endl;
              ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZ >);
              if ( loadPCD< ::pcl::PointXYZ >(filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZ cloud.");
                return 1;
              }
              std::cout << "Made it this far" << std::endl;
              PointCloud p( cloud );
              *cloud_ = p;
            } break;
          case FORMAT_XYZRGB:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
              if ( loadPCD< ::pcl::PointXYZRGB > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZRGB cloud.");
                return 1;
              }
              PointCloud p( cloud );
              *cloud_ = p;
            } break;
          case FORMAT_XYZRGBL:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGBL >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGBL >);
              if ( loadPCD< ::pcl::PointXYZRGBL > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZRGBL cloud.");
                return 1;
              }
              PointCloud p( cloud );
              *cloud_ = p;
            } break;

          // Feature cloud cases
          case FORMAT_PFHSIGNATURE:
            {
              ::pcl::PointCloud< ::pcl::PFHSignature125 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PFHSignature125 >);
              if ( loadPCD< ::pcl::PFHSignature125 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PFHSignature125 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_FPFHSIGNATURE:
            {
              ::pcl::PointCloud< ::pcl::FPFHSignature33 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::FPFHSignature33 >);
              if ( loadPCD< ::pcl::FPFHSignature33 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read FPFHSignature33 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_VFHSIGNATURE:
            {
              ::pcl::PointCloud< ::pcl::VFHSignature308 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::VFHSignature308 >);
              if ( loadPCD< ::pcl::VFHSignature308 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read VFHSignature308 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_SHOT:
            {
              ::pcl::PointCloud< ::pcl::SHOT352 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::SHOT352 >);
              if ( loadPCD< ::pcl::SHOT352 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read SHOT352 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_UNIQUESHAPECONTEXT:
            {
              ::pcl::PointCloud< ::pcl::UniqueShapeContext1960 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::UniqueShapeContext1960 >);
              if ( loadPCD< ::pcl::UniqueShapeContext1960 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read UniqueShapeContext1960 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_ESFSIGNATURE:
            {
              ::pcl::PointCloud< ::pcl::ESFSignature640 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::ESFSignature640 >);
              if ( loadPCD< ::pcl::ESFSignature640 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read ESFSignature640 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_SHAPECONTEXT:
            {
              ::pcl::PointCloud< ::pcl::ShapeContext1980 >::Ptr cloud (new ::pcl::PointCloud< ::pcl::ShapeContext1980 >);
              if ( loadPCD< ::pcl::ShapeContext1980 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read ShapeContext1980 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_GRSDSIGNATURE:
            {
              ::pcl::PointCloud< ::pcl::GRSDSignature21>::Ptr cloud (new ::pcl::PointCloud< ::pcl::GRSDSignature21 >);
              if ( loadPCD< ::pcl::GRSDSignature21 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read GRSDSignature21 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_PRINCIPALRADIIRSD:
            {
              ::pcl::PointCloud< ::pcl::PrincipalRadiiRSD >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PrincipalRadiiRSD >);
              if ( loadPCD< ::pcl::PrincipalRadiiRSD > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PrincipalRadiiRSD cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          case FORMAT_HISTOGRAM153:
            {
              ::pcl::PointCloud< ecto::pcl::Histogram153 >::Ptr cloud (new ::pcl::PointCloud< ecto::pcl::Histogram153 >);
              if ( loadPCD< ecto::pcl::Histogram153 > (filename_, cloud, buffer_) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read ecto::pcl::Histogram153 cloud.");
                return 1;
              }
              FeatureCloud p( cloud );
              *feature_ = p;
            } break;
          // Otherwise
          default:
            throw std::runtime_error("PCDReader: Unknown cloud type.");
        }
        return OK;
      }

      spore<PointCloud> cloud_;
      spore<FeatureCloud> feature_;
      spore<ecto::pcl::Format> format_;
      spore<std::string> filename_;
      spore<bool> buffer_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PCDReader, "PCDReader", "Read a cloud from a PCD file");

