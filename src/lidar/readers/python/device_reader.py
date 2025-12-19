import numpy as np
from innopy.api import DeviceInterface, FrameDataAttributes, GrabType



class DeviceReader:
    def __init__(self):
        self.attr = [FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0)]
        self.di = DeviceInterface(config_file_name='/home/bgr/Desktop/roy_ws/first_autonomous_experiment/BGR-PM/Sensors/Lidar/src/algo/detection/python/om_config.json', is_connect=False)
        self.dataFrame = None

        for i in range(len(self.attr)):
            self.di.activate_buffer(self.attr[i], True)

    def callback(self):
        try:
            res = self.di.get_frame(self.attr)
            if res.success:
                self.dataFrame = res.results['GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0']
                return self.dataFrame
        except:
            print('NewFrameHandler: failed executing frame')


if __name__ == '__main__':
   pass