import numpy as np 
import matplotlib.pyplot as plt  
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import os
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib import cm

class Animation: 
    
    def __init__(self, poses, f_rate=100, fig_x=8, fig_y=8):
        self.poses = poses
        self.f_rate = f_rate
        self.fig = plt.figure(figsize=(fig_x, fig_y), dpi=150, facecolor='black')
        self.ax0 = self.fig.add_subplot(111, projection='3d')
        self.quivers = []
        self.setup_plot()
        self.draw_speed_path() 
        
    def setup_plot(self):
        self.ax0.set_facecolor('black')
        self.ax0.set_xlim(-3, 4)
        self.ax0.set_ylim(-3, 4)
        self.ax0.set_zlim(-3, 4)
        self.ax0.set_xlabel('X')
        self.ax0.set_ylabel('Y')
        self.ax0.set_zlabel('Z')
        self.ax0.tick_params(colors='white')
        self.ax0.xaxis.label.set_color('white')
        self.ax0.yaxis.label.set_color('white')
        self.ax0.zaxis.label.set_color('white')
        self.ax0.title.set_color('white')
        self.ax0.set_title("SE3 Pose Transformation + Velocity")

    def update_plot(self, i):
        for quiv in self.quivers:
            quiv.remove()
        self.quivers = []

        pos = self.poses[i].pos
        rot = self.poses[i].rot_mat
        vel = self.poses[i].velocity

        length = 2.0
        origin = pos
        x_axis = rot[:, 0] * length
        y_axis = rot[:, 1] * length
        z_axis = rot[:, 2] * length

        self.quivers.append(self.ax0.quiver(*origin, *x_axis, color='r', linewidth=2))
        self.quivers.append(self.ax0.quiver(*origin, *y_axis, color='g', linewidth=2))
        self.quivers.append(self.ax0.quiver(*origin, *z_axis, color='b', linewidth=2))

        self.quivers.append(self.ax0.quiver(*origin, *vel, color='m', linewidth=1.5))

        return self.quivers

    def draw_speed_path(self):
        positions = np.array([pose.pos for pose in self.poses])
        speeds = np.array([pose.speed for pose in self.poses])

        norm_speeds = (speeds - speeds.min()) / (speeds.max() - speeds.min() + 1e-6)
        cmap = cm.get_cmap("jet")  

        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            seg = np.vstack((p1, p2)).T
            color = cmap(norm_speeds[i])
            self.ax0.plot(*seg, color=color, linewidth=2)
            
        norm = Normalize(vmin=speeds.min(), vmax=speeds.max())
        sm = ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=self.ax0, shrink=0.5, pad=0.1)
        cbar.set_label("Speed Magnitude", color='white')
        cbar.ax.yaxis.set_tick_params(color='white')
        plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')

    def animate(self, save=True):
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            frames=len(self.poses),
            interval=50,
            repeat=True,
            blit=False 
        )

        if save:
            os.makedirs("assets", exist_ok=True)
            ani.save("assets/transformation.gif", writer="pillow", fps=20)
            print("Saved animation to assets/transformation.gif")

        # plt.show()
