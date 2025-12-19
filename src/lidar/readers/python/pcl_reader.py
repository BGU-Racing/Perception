import numpy as np
from innopy.api import FileReader, FrameDataAttributes, GrabType
import time

class PCL_Reader:
    def __init__(self, recording_path,frame_skip=1, start_frame_num=0):
        self.recording_path = "../ut/" + recording_path
        print("recording path: ", self.recording_path)
        self.attr = [FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0)]
        self.fr = FileReader(recording_path)
        self.number_of_frames = self.fr.num_of_frames
        # self.number_of_frames = 100000000

        self.current_frame_num = start_frame_num
        self.data_frame = None
        self.points = []
        self.frame_skip = frame_skip

    def get_frame(self):
        if self.current_frame_num > self.number_of_frames:
            raise Exception("No more frames to read, end of recording")
        res = self.fr.get_frame(self.current_frame_num, self.attr)
        while res.success is False:
            print("Couldn't get frame number: ", self.current_frame_num)
            self.current_frame_num += 1
            print("Try to get frame number: ", self.current_frame_num)
            res = self.fr.get_frame(self.current_frame_num, self.attr)
        # self.current_frame_num += self.frame_skip
        self.data_frame = res.results['GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0']
        self.current_frame_num += 1
    def basic_reader_filter(self):
        start = time.time()
        mask = self.data_frame['confidence'] > 61
        coords = self.data_frame[mask]
        self.points = np.stack([coords['x'], coords['y'], coords['z']], axis=1).tolist()
        print("Basic reader filter time: ", time.time() - start)

    def read_points_cloud(self):
        self.get_frame()
        self.basic_reader_filter()
        return np.array(self.points)
