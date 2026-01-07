import open3d as o3d
import keyboard
import mouse


class Visualizer:
    def __init__(self,visualizer="open3d"):
        print("Visualizer: ",visualizer)
        if not visualizer == "pptk" and not visualizer=="open3d":
            raise Exception("Visualizer not supported")
        self.visualizer = visualizer
        self.v = None

    def show(self,points):
        if self.visualizer == "open3d":
            self.v = o3d.geometry.PointCloud()
            self.v.points = o3d.utility.Vector3dVector(points)
            o3d.visualization.draw_geometries([self.v])

    def close(self):
        if self.visualizer == "pptk":
            self.v.close()
        elif self.visualizer == "open3d":
            self.v.close()

    def get_points(self, points):
        if self.visualizer == 'open3d':
            raise Exception('Open3d point selection is not supported.')
        self.show(points)
        while True:
            if keyboard.read_key() == 'z':
                selected_points = self.v.get('selected')
                print(selected_points)
                mouse.right_click()
                self.v.close()
                if len(selected_points) == 3:
                    confirmation = input("Are you sure you have picked the right points? y/n\n")
                    if confirmation == 'y':
                        return selected_points
                print("Something went wrong - choose points again!")
                self.show(points)
