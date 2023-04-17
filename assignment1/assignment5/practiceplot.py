import matplotlib.pyplot as mpl
import numpy as np
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from stl import mesh

fig = mpl.figure()
ax = fig.add_subplot(111, projection='3d')

# Cylinder
#x=np.linspace(-1, 1, 50)
#z=np.linspace(-2, 2, 50)
#Xc, Zc=np.meshgrid(x, z)
#Yc = np.sqrt(1-Xc**2)

angle = np.linspace(0,2*np.pi,36)
v = np.linspace(0,3,12)
Xc = np.cos(angle)
Yc = np.sin(angle)
a,Zc = np.meshgrid(angle,v)

v = np.linspace(3,6,12)
Xc2 = 0.3*np.cos(angle)
Yc2 = 0.3*np.sin(angle)
a,Zc2 = np.meshgrid(angle,v)

v = np.linspace(-3,0,12)
Xc3 = np.linspace(-2,2,20)
Yc3 = np.linspace(-2,2,20)
a, Zc3 = np.meshgrid(Xc3,v)



# Draw parameters
#rstride = 20
#cstride = 10
#ax.plot_surface(Xc, Yc, Zc, alpha=0.2, rstride=rstride, cstride=cstride)
#ax.plot_surface(Xc, -Yc, Zc, alpha=0.2, rstride=rstride, cstride=cstride)
#ax.plot_surface(Xc,Yc,Zc)
#ax.plot_surface(Xc2,Yc2,Zc2)
#ax.plot_surface(Xc3,Yc3,Zc3)

my_mesh = mesh.Mesh.from_file('assignment1/assignment5/20mm_cube.stl')
#print(my_mesh.vectors)
my_mesh.translate([-10,-10,-10])
#print(my_mesh.vectors)
print(my_mesh.x)
print(my_mesh.y)
print(my_mesh.z)

ax.add_collection3d(mplot3d.art3d.Poly3DCollection(my_mesh.vectors))

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
mpl.show()

print("woah")

def cylinder(r, h, a =0, nt=100, nv =50):
    """
    parametrize the cylinder of radius r, height h, base point a
    """
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    z = v
    return x, y, z