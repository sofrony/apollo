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


int connection_loop(OS1::client& cli) {

    //std::ofstream out;
    //out.open("/apollo/lidar_buf.txt");

    std::shared_ptr<apollo::cyber::Node> os_node(apollo::cyber::CreateNode("ouster_talker"));
    auto os_talker = os_node->CreateWriter<apollo::drivers::PointCloud>("/apollo/poka_chto_ne_lidar");
    
    std::vector<uint8_t> lidar_packet, imu_packet;

    lidar_packet.resize(OS1::lidar_packet_bytes + 1);
    imu_packet.resize(OS1::imu_packet_bytes + 1);

    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string("1024x10"));

    auto xyz_lut = OS1::make_xyz_lut(W, H, beam_azimuth_angles, beam_altitude_angles);

    CloudOS1 cloud{W, H};
    auto it = cloud.begin();

    //std::shared_ptr<apollo::cyber::base::CCObjectPool<apollo::drivers::PointCloud>> point_cloud_pool_ = nullptr;
    
    //std::shared_ptr<apollo::drivers::PointCloud> apollo_pc = point_cloud_pool_->GetObject();
    //auto apollo_pc = std::shared_ptr<apollo::drivers::PointCloud>();
    /*
    apollo_pc->mutable_header()->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
    apollo_pc->mutable_header()->set_frame_id("ouster");
    apollo_pc->mutable_header()->set_lidar_timestamp(apollo::cyber::Time::Now().ToSecond());
    apollo_pc->set_measurement_time(apollo::cyber::Time::Now().ToSecond());
    apollo_pc->set_height(1);
    apollo_pc->set_width(11000);
    apollo_pc->set_is_dense(true);
    apollo_pc->mutable_point()->Reserve(240000);
    */
    auto batch_and_publish = OS1::batch_to_iter<CloudOS1::iterator>(
                    xyz_lut, W, H, {}, &PointOS1::make,
                    [&](uint64_t scan_ts) mutable {
                        AERROR << "FFFFF";
                        //for(int iii=0;iii<=11000;iii++){
                            //apollo_pc->set_measurement_time(0);
                            /*
                            apollo::drivers::PointXYZIT* cyber_point = apollo_pc->add_point();
                            cyber_point->set_timestamp(0);
                            cyber_point->set_x(static_cast<float>(0.0));
                            cyber_point->set_y(static_cast<float>(1.0));
                            cyber_point->set_z(static_cast<float>(2.0));
                            cyber_point->set_intensity(0);
                            */
                        //}
                        //msg = cloud_to_cyber(cloud);
                        //os_talker->Write(apollo_pc);
                        });

    while (apollo::cyber::OK()){
        auto state = OS1::poll_client(cli);
        if (state == OS1::EXIT) {
            AINFO << "poll_client: caught signal, exiting";
            return EXIT_SUCCESS;
        }
        if (state & OS1::ERROR) {
            AERROR << "poll_client: returned error";
            return EXIT_FAILURE;
        }
        if (state & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(cli, lidar_packet.data()))
                //AERROR << "GOT LIDAR";
                AERROR << lidar_packet.data();
                batch_and_publish(lidar_packet.data(), it);
        }
        /*
        if (state & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(cli, imu_packet.data()))
                //AERROR << "GOT IMU";
        }
        */
    }
    return EXIT_SUCCESS;
}

int main(int argc, char** argv) {

    /*
    if (argc != 3) {
        std::cerr << "Usage: ouster_cyber <os1_hostname> "
                     "<data_destination_ip>"
                  << std::endl;
        return 1;
    }
    */

    apollo::cyber::Init(argv[0]);

    auto cli = OS1::init_client("10.5.5.77", "10.5.5.1", OS1::lidar_mode_of_string("1024x10"), 7501, 7502);

    if (!cli) {
        std::cerr << "Failed to connect to client at: " << argv[1] << std::endl;
        return 1;
    }

    auto metadata = OS1::get_metadata(*cli);
    std::ofstream meta_out;
    meta_out.open("/apollo/lidar_meta.txt");
    meta_out << metadata;
    meta_out.close();

    return connection_loop(*cli);

    return 0;
}
