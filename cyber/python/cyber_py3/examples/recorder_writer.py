import time
import math
import os,sys
from google.protobuf.descriptor_pb2 import FileDescriptorProto

from cyber.proto.unit_test_pb2 import Chatter
from cyber.python.cyber_py3 import record
from modules.common.util.testdata.simple_pb2 import SimpleMessage

## CyberRT messages
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles


Chassis_msg = "apollo.canbus.Chassis"
Localization_msg = "apollo.localization.LocalizationEstimate"
Obstacle_msg = "apollo.perception.PerceptionObstacles"
Transform_msg = "apollo.transform.TransformStampeds"
Control_msg = "apollo.control.ControlCommand"

def test_record_creater(writer_path):
    """
    Record writer.
    """
    fwriter = record.RecordWriter()
    fwriter.set_size_fileseg(0)
    fwriter.set_intervaltime_fileseg(0)

    if not fwriter.open(writer_path):
        print('Failed to open record writer!')
        return
    print('+++ Begin to writer +++')

    msg_new = Chassis()
    desc_str = msg_new.SerializeToString()
    fwriter.write_channel("/apollo/canbus/chassis",Chassis_msg , desc_str)
    # fwriter.write_channel('/apollo/localization/pose',Localization_msg , proto_desc)
    # fwriter.write_channel('/apollo/perception/obstacles', Obstacle_msg, proto_desc)
    # fwriter.write_channel('/tf', Transform_msg, proto_desc)
    # fwriter.write_message('simplemsg_channel', , proto_desc)
    # fwriter.write_message('simplemsg_channel', , proto_desc)

    

    fwriter.close()








if __name__ == '__main__':
    
    test_record_file = "/apollo/self_test.record"

    print('Begin to write record file: {}'.format(test_record_file))
    test_record_creater(test_record_file)