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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/variant/get.hpp>

#include <X11/Xlib.h>

#include <sstream>
#include <map>


//PCL_VISUALIZER_POINT_SIZE
//PCL_VISUALIZER_OPACITY
//PCL_VISUALIZER_LINE_WIDTH
//PCL_VISUALIZER_FONT_SIZE
//PCL_VISUALIZER_COLOR  (3 values 0.0 --> 1.0)
//PCL_VISUALIZER_REPRESENTATION
//PCL_VISUALIZER_IMMEDIATE_RENDERING
//PCL_VISUALIZER_SHADING
//PCL_VISUALIZER_LUT
//PCL_VISUALIZER_LUT_RANGE

namespace ecto
{
  namespace pcl
  {
    using ::pcl::visualization::PCLVisualizer;
    struct CloudViewer
    {
      // A map to store properties for visualize:
      //           * First key is aimed to refer to the cloud
      //           * Second key is aimed to refer to the property
      //           * value
      typedef std::map<int, std::vector<double> > Property;
      typedef std::map<std::string, Property > Properties;

      CloudViewer() : num_inputs(1), use_ports(false)
      {
        XInitThreads();
      }

      static void
      declare_params(tendrils& params)
      {
        std::vector<int> bc_def; bc_def.push_back(0), bc_def.push_back(0), bc_def.push_back(255);
        params.declare<std::string>("window_name", "The window name", "cloud viewer");
        params.declare<int>("num_inputs", "Numb of input clouds to visualize", 1);
        params.declare<Properties>("rendering_properties", "");
        params.declare<std::vector<int> >("bc", "Background color", bc_def);
        params.declare<bool> ("use_ports", "Use different ports for each cloud", false);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<bool>("__quit__", "Flag intendend to quit the process.", false);

        int n_i = 1;
        params["num_inputs"] >> n_i;
        for (int i=1; i <= n_i; i++)
        {
          std::stringstream ss;
          ss << "input" << i;
          inputs.declare<PointCloud>(ss.str(), "The cloud to view");
        }
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        params["window_name"] >> window_name;
        params["num_inputs"] >> num_inputs;
        params["rendering_properties"] >> rendering_properties;
        params["bc"] >> bc;
        params["use_ports"] >> use_ports;
      }

      void
      run()
      {
        viewer_.reset(new PCLVisualizer(window_name));
        viewer_->initCameraParameters();

        if (use_ports)
        {
          ports = std::vector<int> (num_inputs, 0);
          float step = ports.size();
          for (int i = 0; i < ports.size(); i ++)
          {
            viewer_->createViewPort(1.0/step * ((float)i), 0.0, 1.0/step * ((float)i + 1.0), 1.0, ports[i]);
            viewer_->setBackgroundColor(bc[0], bc[1], bc[2], ports[i]);
          }
        }
        else
        {
          int port(0);
          viewer_->createViewPort(0.0, 0.0, 1.0, 1.0, port);
          viewer_->setBackgroundColor(bc[0], bc[1], bc[2]);
          ports = std::vector<int> (num_inputs, port);
        }

        {
          // TODO: Expose this
          viewer_->addCoordinateSystem(0.25);
          //        // z-axis as a up vector
          //        viewer_->setCameraPosition(-0.806389, 1.36983, 0.297887, /* position*/
          //                                   0.383674, 0.519189, 0.359116, /* focal point*/
          //                                   -0.0112215, 0.056162, 0.998359 /* view up*/
          //                                   );
          //
          // z-axis as a fordward vector
          viewer_->setCameraPosition(-0.123668, 0.0368707, -1.18417, /* position*/
                                     -0.127353, 0.0151198, 0.27976, /* focal point*/
                                     0.0164321, -0.999755, -0.0148128 /* view up*/
          );
          viewer_->setCameraClipDistances(0.001, 100);
        }

        while (!viewer_->wasStopped() && !boost::this_thread::interruption_requested())
        {
          {
            boost::mutex::scoped_try_lock lock(mtx);
            if (lock)
            {
              signal_();
              jobs_.clear(); //disconnects all the slots.
            }
          }
          viewer_->spinOnce(20);
        }
        if (!viewer_->wasStopped())
          viewer_->close();
      }

      struct show_dispatch: boost::static_visitor<>
      {
        show_dispatch(boost::shared_ptr<PCLVisualizer> viewer, const std::string& key, const Property& prop, int port)
                : viewer(viewer)
                  , key(key)
                  , rend_prop(prop)
                  , port(port)
                  {
                  }

        //http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
        template<typename Point>
        void
        operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud) const
        {
          if (!viewer->updatePointCloud<Point>(cloud, key))
          {
            viewer->addPointCloud<Point>(cloud, key, port);
          }
        }
        void
        operator()(boost::shared_ptr<const CloudPOINTXYZRGB>& cloud) const
        {
          ::pcl::visualization::PointCloudColorHandlerRGBField<CloudPOINTXYZRGB::PointType> rgb(cloud);
          if (!viewer->updatePointCloud(cloud, rgb, key))
          {
            viewer->addPointCloud(cloud, rgb, key, port);
          }
          // Set PointCloud properties to render
          for (std::map<int, std::vector<double> >::const_iterator it=rend_prop.begin(); it!=rend_prop.end(); ++it)
          {
            if (it->second.size() > 1)
              viewer->setPointCloudRenderingProperties (it->first, it->second[0], it->second[1], it->second[2], key);
            else
              viewer->setPointCloudRenderingProperties (it->first, it->second[0], key);
          }
        }
        void
        operator()(boost::shared_ptr<const CloudPOINTXYZRGBNORMAL>& cloud) const
        {
          ::pcl::visualization::PointCloudColorHandlerRGBField<CloudPOINTXYZRGBNORMAL::PointType> rgb(cloud);
          //          ::pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<CloudPOINTXYZRGBNORMAL::PointType> normals(
          //              cloud);

          if (!viewer->updatePointCloud(cloud, rgb, key))
          {
            viewer->addPointCloud(cloud, rgb, key, port);
          }
          //          viewer->updatePointCloud(cloud,normals,key);

          boost::this_thread::sleep(boost::posix_time::milliseconds(30));
        }

        boost::shared_ptr<PCLVisualizer> viewer;
        std::string key;
        Property rend_prop;
        int port;
      };

      struct show_dispatch_runner
      {
        show_dispatch_runner(const show_dispatch& dispatch, const xyz_cloud_variant_t& varient)
        :
          dispatch(dispatch),
          varient(varient)
        {
        }
        void
        operator()()
        {
          boost::apply_visitor(dispatch, varient);
        }
        show_dispatch dispatch;
        xyz_cloud_variant_t varient;
      };

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        if (inputs.get<bool>("__quit__"))
        {
          viewer_->close();
//          runner_thread_->interrupt();
          runner_thread_->join();
          return ecto::QUIT;
        }
        if (!runner_thread_)
        {
          runner_thread_.reset(new boost::thread(boost::bind(&CloudViewer::run, this)));
        }
        while (!viewer_)
        {
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }

        {
          boost::this_thread::sleep(boost::posix_time::milliseconds(30));
          boost::mutex::scoped_try_lock lock(mtx);
          if (lock)
          {
            ECTO_SCOPED_CALLPYTHON();

            for (int i=1; i <= num_inputs; i++)
            {
              std::stringstream key;
              key << "input" << i;

              // Prevent to visualize an empty input cloud
              PointCloud cloud = inputs.get<PointCloud>(key.str());
              if (!cloud.held)
                continue;

              xyz_cloud_variant_t varient = cloud.make_variant();
              show_dispatch dispatch(viewer_, key.str(), rendering_properties[key.str()], ports[i-1]);
              boost::shared_ptr<boost::signals2::scoped_connection> c(new boost::signals2::scoped_connection);
              *c = signal_.connect(show_dispatch_runner(dispatch, varient));
              jobs_.push_back(c);
            }
          }
        }

        return ecto::OK;
      }

      ~CloudViewer()
      {
        if (runner_thread_ && runner_thread_->joinable())
        {
          runner_thread_->interrupt();
          runner_thread_->join();
        }
      }

      Properties rendering_properties;
      std::vector<int> bc;
      int num_inputs;
      std::string window_name;
      boost::shared_ptr<PCLVisualizer> viewer_;
      boost::shared_ptr<boost::thread> runner_thread_;
      boost::signals2::signal<void (void)> signal_;
      std::vector<boost::shared_ptr<boost::signals2::scoped_connection> > jobs_;
      boost::mutex mtx;
      bool use_ports;
      std::vector<int> ports;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::CloudViewer, "CloudViewer", "Viewer of clouds");

