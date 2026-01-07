import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from innopy.api import DeviceInterface, FrameDataAttributes, GrabType



class DeviceReader:
    def __init__(self):
        self.attr = [FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0)]
        
        pkg_share = get_package_share_directory("perception")
        config_path = os.path.join(
            pkg_share,
            "utils",
            "om_config.json"
        )

        self.di = DeviceInterface(
            config_file_name=config_path,
            is_connect=False
        )
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