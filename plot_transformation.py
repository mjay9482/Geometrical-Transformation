import numpy as np 
import matplotlib.pyplot as plt  
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

class Animation: 
    
    def __init__(self, poses, f_rate=200, fig_x=12, fig_y=12):
        self.poses = poses
        self.f_rate = f_rate
        self.fig = plt.figure(figsize = (fig_x, fig_y), dpi=200, facecolor = 'black')
        self.ax0 = self.fig.add_subplot(111, projection='3d')
        self.marker, = self.ax0.plot([],[],[],'ro',markersize=6)
        self.quiver = None
        self.setup_plot()
        
    def setup_plot(self):
        self.ax0.set_facecolor('black')
        self.ax0.set_xlim(-2,4)
        self.ax0.set_ylim(-2,4)
        self.ax0.set_zlim(-2,4)
        self.ax0.set_xlabel('X')
        self.ax0.set_ylabel('Y')
        self.ax0.set_zlabel('Z')
        self.ax0.tick_params(colors='white')
        self.ax0.xaxis.label.set_color('white')
        self.ax0.yaxis.label.set_color('white')
        self.ax0.zaxis.label.set_color('white')
        self.ax0.title.set_color('white')
        self.ax0.set_title("SE3 Pose Transformation")
        
    def update_plot(self,i):
        self.ax0.collections.clear()
        pos= self.poses[i].pos
        rot= self.poses[i].rot_mat
        length = 2 
        origin = pos 
        
        x_axis = rot[:,0] * length 
        y_axis = rot[:,1] * length 
        z_axis = rot[:,2] * length 
        
        self.ax0.quiver(*origin, *x_axis, color = 'r', linewidth=2)
        self.ax0.quiver(*origin, *y_axis, color = 'g', linewidth=2)
        self.ax0.quiver(*origin, *z_axis, color = 'b', linewidth=2)
        
        return []
    
    
    def animate(self):
        ani = animation.FuncAnimation(self.fig,
                                      self.update_plot,
                                      frames = self.f_rate,
                                      interval = 30,
                                      repeat = True,
                                      blit = True)
        plt.show()