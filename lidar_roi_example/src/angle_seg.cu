#include "lidar_roi_example/angle_seg.h"

#include "assert.h"
#include "ros/ros.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>


namespace ROIExample
{
    inline cudaError_t checkCuda(cudaError_t result)
    {
        if (result != cudaSuccess)
        {
            fprintf(stderr, "CUDA Runtime Error: %s\n", cudaGetErrorString(result));
            assert(result == cudaSuccess);
        }
        return result;
    }

    __global__ void AzimuthSeg(pcl::PointXYZ::Ptr d_pc_in, pcl::PointXYZ::Ptr d_pc_prj)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (!(idx % 3))
        {
            float &x = (*(d_pc_in)[idx]).x;
            float &y = (*(d_pc_in)[idx]).y;
            float &z = (*(d_pc_in)[idx]).z;

            float theta = atan2(x, -y) * 180 / M_PI - 90;
            theta = (theta < 0) ? theta + 360 : theta;

            if (fabs(theta - 30) < 5)
            {
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                d_pc_prj->push_back(point);
            }
        }
    }
    
    void AngleSeg::Init(pcl::PointCloud<pcl::PointXYZ> *&input)
    {
        auto start = ros::Time::now();

        // allocate memory for device
        int size = input->points.size();
        thrust::device_vector<pcl::PointXYZ> d_pc_in;
        thrust::device_vector<pcl::PointXYZ> d_pc_prj;

        pcl::PointCloud<pcl::PointXYZ>* pc_seg;

        // checkCuda(cudaMallocManaged(&d_pc_in, sizeof(pcl::PointXYZ) * size));
        // checkCuda(cudaMallocManaged(&d_pc_prj, sizeof(pcl::PointXYZ) * size));

        // copy pointcloud data to device
        checkCuda(cudaMemcpy(d_pc_in, input->points, sizeof(pcl::PointXYZ) * size, cudaMemcpyHostToDevice));
        pcl::PointXYZ::Ptr d_pc_ptr = thrust::raw_pointer_cast(d_pc_in.data);
        pcl::PointXYZ::Ptr d_pc_prj_ptr = thrust::raw_pointer_cast(d_pc_prj.data);

        // 170 blocks, 1020 threads
        // 1020/3 = 340 points per block
        // 170 x 340 = 57800 maximum points
        AzimuthSeg<<<170, 1020>>>(d_pc_in, d_pc_prj);

        checkCuda(cudaMemcpy(pc_seg->points, d_pc_prj, sizeof(pcl::PointXYZ) * size, cudaMemcpyHostToDevice));
        pc_seg_ = pc_seg;

        auto end = ros::Time::now();
        ROS_WARN("time %f", (end - start).toSec());
    }

    pcl::PointCloud<pcl::PointXYZ>* AngleSeg::GetPC()
    {
        return pc_seg_;
    }
}