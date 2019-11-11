load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ouster_test",
    srcs = [
        "src/test.cpp",
        "src/os1.cpp",
        "src/os1_util.cpp",
        "include/ouster/os1_packet.h",
        "include/ouster/os1_util.h",
        "include/ouster/os1.h",
    ],
    deps = [
        "//third_party/ouster_json",
    ],
)

cc_binary(
    name = "ouster_cyber",
    srcs = [
        "src/main.cpp",
        "src/os1.cpp",
        "src/os1_util.cpp",
        "include/ouster/os1_packet.h",
        "include/ouster/os1_util.h",
        "include/ouster/os1.h",
        "include/ouster/point_os1.h",
    ],
    deps = [
        "//third_party/ouster_json",
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/transform:tf2_buffer_lib",
        "@pcl",
        "@eigen",
    ],
)

cc_binary(
    name = "ouster_cyber_from_log",
    srcs = [
        "src/main_read_from_buf.cpp",
        "src/os1.cpp",
        "src/os1_util.cpp",
        "include/ouster/os1_packet.h",
        "include/ouster/os1_util.h",
        "include/ouster/os1.h",
        "include/ouster/point_os1.h",
    ],
    deps = [
        "//third_party/ouster_json",
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/transform:tf2_buffer_lib",
        "@pcl",
        "@eigen",
    ],
)

cpplint()