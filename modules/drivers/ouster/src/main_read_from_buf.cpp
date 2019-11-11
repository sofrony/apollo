#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include "modules/drivers/ouster/include/ouster/os1.h"
#include "modules/drivers/ouster/include/ouster/os1_packet.h"
#include "modules/drivers/ouster/include/ouster/os1_util.h"
#include "modules/drivers/ouster/include/ouster/point_os1.h"

#include "modules/transform/buffer.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/base/concurrent_object_pool.h"
#include "cyber/base/object_pool.h"

//#include "modules/drivers/velodyne/proto/config.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/common/util/message_util.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;

namespace OS1 = ouster::OS1;

using PointOS1 = ouster::OS1::PointOS1;
using CloudOS1 = pcl::PointCloud<PointOS1>;
using namespace apollo::cyber;
using namespace apollo::drivers;
using namespace apollo::cyber::base;

const std::vector<double> beam_altitude_angles = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

const std::vector<double> beam_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

uint64_t n_lidar_packets = 0;
uint64_t n_imu_packets = 0;

uint64_t lidar_col_0_ts = 0;
uint64_t imu_ts = 0;

float lidar_col_0_h_angle = 0.0;
float imu_av_z = 0.0;
float imu_la_y = 0.0;
std::shared_ptr<Writer<PointCloud>> writer_;
std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
int pool_size_ = 8;

bool Init() {
    std::shared_ptr<apollo::cyber::Node> node_(apollo::cyber::CreateNode("ouster_talker"));
    writer_ =
      node_->CreateWriter<PointCloud>("/apollo/poka_chto_ne_lidar");
  point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(140000);
  }
  AINFO << "Point cloud comp convert init success";
  return true;
}

bool Proc() {
  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
  if (point_cloud_out == nullptr) {
    AWARN << "poin cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<PointCloud>();
    point_cloud_out->mutable_point()->Reserve(140000);
  }
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return false;
  }
  point_cloud_out->Clear();
  apollo::drivers::PointXYZIT* cyber_point = point_cloud_out->add_point();
  cyber_point->set_timestamp(0);
  cyber_point->set_x(static_cast<float>(0.0));
  cyber_point->set_y(static_cast<float>(1.0));
  cyber_point->set_z(static_cast<float>(2.0));
  cyber_point->set_intensity(0);
  point_cloud_out->set_measurement_time(0);

  if (point_cloud_out == nullptr || point_cloud_out->point().empty()) {
    AWARN << "point_cloud_out convert is empty.";
    return false;
  }
  writer_->Write(point_cloud_out);
  return true;
}


int connection_loop() {
    std::vector<uint8_t> lidar_packet, imu_packet;

    lidar_packet.resize(OS1::lidar_packet_bytes + 1);
    imu_packet.resize(OS1::imu_packet_bytes + 1);

    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string("1024x10"));

    auto xyz_lut = OS1::make_xyz_lut(W, H, beam_azimuth_angles, beam_altitude_angles);

    CloudOS1 cloud{W, H};
//    auto it = cloud.begin();

    Init();
    while (apollo::cyber::OK()){
        std::ifstream lidar_in("/apollo/lidar_buf.txt");
        std::string lidar_packet_line;
        while(getline(lidar_in, lidar_packet_line)){
            //for(auto i : lidar_packet_line) 
            //std::cout << lidar_packet_line;
            Proc();
        }
        lidar_in.close();
    }
    return 0;
}

int main(int argc, char** argv) {

    apollo::cyber::Init(argv[0]);
    
    AERROR << "Reading from file /apollo/lidar_buf.txt";
    return connection_loop();

    return 0;
}
