#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>

namespace ROIExample
{
    class AngleSeg
    {
    public:
        void Init(pcl::PointCloud<pcl::PointXYZ>* &input);
        pcl::PointCloud<pcl::PointXYZ>* GetPC();

    private:
        pcl::PointCloud<pcl::PointXYZ>* pc_seg_;
    };
}