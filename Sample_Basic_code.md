

# Modeling Algorithms for Agricultural Data

# Iterative Closest Point (ICP) basic c++ code 

**Student**: Omia Emmanuel

**Instructor**: Prof. Lee Wang-hee

This code demonstrates using the Iterative Closest Point algorithm to determine if one PointCloud is just a rigid transformation of another by minimizing the distances between the points of two pointclouds and rigidly transforming them


## The Code

### Warning: You must build [pcl](https://pointclouds.org/) first if you want to run the example.  
```cpp

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  for (auto& point : *cloud_in)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
      
  for (auto& point : *cloud_in)
    std::cout << point << std::endl;
      
  *cloud_out = *cloud_in;
  
  std::cout << "size:" << cloud_out->size() << std::endl;
  for (auto& point : *cloud_out)
    point.x += 0.7f;

  std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
  for (auto& point : *cloud_out)
    std::cout << point << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}

```

## The explanation

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
```
These are the header files that contain the definitions for all of the classes which we will use.

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
```
Creates two pcl::PointCloud<pcl::PointXYZ> boost shared pointers and initializes them. The type of each point is set to PointXYZ in the pcl namespace which is:

```cpp
struct PointXYZ
{
  float x;
  float y;
  float z;
};
```
Brief point structure representing Euclidean xyz coordinates.

```cpp
// Fill in the CloudIn data
  for (auto& point : *cloud_in)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
      
  for (auto& point : *cloud_in)
    std::cout << point << std::endl;
      
  *cloud_out = *cloud_in;
  
  std::cout << "size:" << cloud_out->size() << std::endl;
  for (auto& point : *cloud_out)
    point.x += 0.7f;
```

Fill in the PointCloud structure with random point values, and set the appropriate parameters (width, height, is_dense). Also, they output the number of points saved, and their actual data values.

Then:

```cpp
std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
  for (auto& point : *cloud_out)
    std::cout << point << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
```

performs a simple rigid transform on the pointcloud and again outputs the data values.

```cpp
icp.setInputSource(cloud_in);
icp.setInputTarget(cloud_out);
```
This creates an instance of an IterativeClosestPoint and gives it some useful information. “icp.setInputSource(cloud_in);” sets cloud_in as the PointCloud to begin from and “icp.setInputTarget(cloud_out);” sets cloud_out as the PointCloud which we want cloud_in to look like.

Next,

```cpp
pcl::PointCloud<pcl::PointXYZ> Final;
icp.align(Final);

std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " <<
icp.getFitnessScore() << std::endl;
```
Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm. If the two PointClouds align correctly (meaning they are both the same cloud merely with some kind of rigid transformation applied to one of them) then icp.hasConverged() = 1 (true). It then outputs the fitness score of the final transformation and some information about it.

## Output

```cpp
Saved 5 data points to input:
 0.352222 -0.151883 -0.106395
-0.397406 -0.473106 0.292602
-0.731898 0.667105 0.441304
-0.734766 0.854581 -0.0361733
-0.4607 -0.277468 -0.916762
size:5
Transformed 5 data points:
 1.05222 -0.151883 -0.106395
 0.302594 -0.473106 0.292602
-0.0318983 0.667105 0.441304
-0.0347655 0.854581 -0.0361733
      0.2393 -0.277468 -0.916762
[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample
selection distance threshold of: 0.200928
[pcl::IterativeClosestPoint::computeTransformation] Number of
correspondences 4 [80.000000%] out of 5 points [100.0%], RANSAC rejected:
1 [20.000000%].
[pcl::IterativeClosestPoint::computeTransformation] Convergence reached.
Number of iterations: 1 out of 0. Transformation difference: 0.700001
ICP has converged, score: 1.95122e-14
          1  4.47035e-08 -3.25963e-09          0.7
2.98023e-08            1 -1.08499e-07 -2.98023e-08
1.30385e-08 -1.67638e-08            1  1.86265e-08
          0            0            0            1

```