set(ECTO_PCL_FILES
  ecto_pcl.cpp
  keypoints/HarrisKeypoint3D.cpp
  features/CVFHEstimation.cpp
  features/ESFEstimation.cpp
  features/FPFHEstimation.cpp
  features/GRSDEstimation.cpp
  features/NormalEstimation.cpp
  features/OURCVFHEstimation.cpp
  features/PFHEstimation.cpp
  features/RSDEstimation.cpp
  features/SHOTEstimation.cpp
  features/SIEstimation.cpp
  features/USCEstimation.cpp
  features/VFHEstimation.cpp
  filters/Cropper.cpp
  filters/ExtractIndices.cpp
  filters/PassThrough.cpp
  filters/PassThroughIndices.cpp
  filters/ProjectInliers.cpp
  filters/RemoveNaNPoints.cpp
  filters/StatisticalOutlier.cpp
  filters/RadiusOutlier.cpp
  filters/VoxelGrid.cpp
  io/CloudViewer.cpp
  io/CopyCloud.cpp
#  io/KinectGrabber.cpp
  io/MergeClouds.cpp
  io/NiConverters.cpp
  io/PCDReader.cpp
  io/PCDWriter.cpp
  io/PLYWriter.cpp
  segmentation/ColorizeClusters.cpp
  segmentation/ExtractPolygonalPrismData.cpp
  segmentation/ExtractLargestCluster.cpp
  segmentation/EuclideanClusterExtraction.cpp
  segmentation/SACSegmentation.cpp
  segmentation/SACSegmentationFromNormals.cpp
  surface/ConvexHull.cpp
  surface/MovingLeastSquares.cpp
)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
add_subdirectory(ros)
else()
  set(ECTO_PCL_FILES ${ECTO_PCL_FILES} io/PCDWriter.cpp)
endif()

ectomodule(ecto_pcl INSTALL ${ECTO_PCL_FILES})

set_source_files_properties(io/CloudViewer.cpp
  PROPERTIES
  COMPILE_FLAGS -Wno-deprecated
  )

link_ecto(ecto_pcl ${PCL_LIBRARIES}
                   ${catkin_LIBRARIES}
)
