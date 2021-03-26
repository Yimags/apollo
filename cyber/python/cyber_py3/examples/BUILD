load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "record",
    srcs = ["record.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/common/proto:error_code_py_pb2",
        "//modules/common/proto:header_py_pb2",
        "//modules/common/util/testdata:simple_py_pb2",

	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
    ],
)

py_binary(
    name = "record_custom",
    srcs = ["record_custom.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/common/proto:error_code_py_pb2",
        "//modules/common/proto:header_py_pb2",
        "//modules/common/util/testdata:simple_py_pb2",

	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
    ],
)

py_binary(
    name = "time",
    srcs = ["time.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
    ],
)

py_binary(
    name = "test",
    srcs = ["test.py"],
    deps = [
        "//cyber/python/cyber_py3/examples:listener",
    ],
)

py_binary(
    name = "timer",
    srcs = ["timer.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_timer",
    ],
)

py_binary(
    name = "talker",
    srcs = ["talker.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "writer_custom",
    srcs = ["writer_custom.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/routing/proto:routing_py_pb2",
    ],
)


py_binary(
    name = "recorder_writer",
    srcs = ["recorder_writer.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
	"//modules/common/util/testdata:simple_py_pb2",

	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
"//modules/drivers/proto:sensor_image_py_pb2",
"//modules/routing/proto:routing_py_pb2",
    ],
)


py_binary(
    name = "listener",
    srcs = ["listener.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
	"//modules/common/util/testdata:simple_py_pb2",

	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
"//modules/drivers/proto:sensor_image_py_pb2",
"//modules/routing/proto:routing_py_pb2",
"//modules/drivers/proto:conti_radar_py_pb2",
"//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
"//modules/localization/proto:gps_py_pb2",
"//modules/drivers/gnss/proto:imu_py_pb2",
    ],
)

py_binary(
    name = "listener_custom",
    srcs = ["listener_custom.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
	"//modules/common/util/testdata:simple_py_pb2",

	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
"//modules/drivers/proto:sensor_image_py_pb2",
"//modules/routing/proto:routing_py_pb2",
"//modules/drivers/proto:pointcloud_py_pb2",
    ],
)

py_binary(
    name = "realtime_data_storage",
    srcs = ["realtime_data_storage.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
	"//modules/common/util/testdata:simple_py_pb2",
        "//cyber/python/cyber_py3:cyber_time",
	"//modules/canbus/proto:chassis_py_pb2",
	"//modules/localization/proto:localization_py_pb2",
	"//modules/perception/proto:perception_obstacle_py_pb2",
	"//modules/transform/proto:transform_py_pb2",
	"//modules/control/proto:control_cmd_py_pb2",
	"//modules/prediction/proto:prediction_obstacle_py_pb2",
	"//modules/drivers/proto:sensor_image_py_pb2",
	"//modules/routing/proto:routing_py_pb2",
	"//modules/drivers/proto:pointcloud_py_pb2",
	"//modules/drivers/proto:conti_radar_py_pb2",
	"//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
	"//modules/localization/proto:gps_py_pb2",
	"//modules/drivers/gnss/proto:imu_py_pb2",
    ],
)

py_binary(
    name = "parameter",
    srcs = ["parameter.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:parameter",
    ],
)

py_binary(
    name = "service",
    srcs = ["service.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "client",
    srcs = ["client.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "record_trans",
    srcs = ["record_trans.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)

py_binary(
    name = "record_channel_info",
    srcs = ["record_channel_info.py"],
    deps = [
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)



py_binary(
    name = "mock_routing_request",
    srcs = ["mock_routing_request.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
    ],
)
