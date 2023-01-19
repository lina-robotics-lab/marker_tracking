import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull
from mpl_toolkits import mplot3d

import sys
sys.path.append('../scripts')

from region import AcquisitionRegion

import pickle as pkl

import geometry_msgs

with open('../data/corners.pkl', 'rb') as f:
    corners = pkl.load(f)


points = np.array([[p.position.x,p.position.y,p.position.z] for p in corners['corner_poses']])
region = AcquisitionRegion(points)



front = np.array(list(range(6)))
faces = [front,front+6]

edge0 = np.array([0,1])
front_edges = [(edge0+i)%6 for i in range(6)]
print('front_edges',front_edges)

side_faces = [np.hstack([f,np.flip(f+6)]) for f in front_edges]
print('side_faces',side_faces)
faces = np.array(faces + side_faces)
print('faces',faces)

# ax = plt.axes(projection='3d')
# for f in faces:
# 	ax.plot3D(points[f,0],points[f,1],points[f,2])

centers = np.array([np.mean(points[f,:],axis = 0) for f in faces])
# ax.scatter3D(centers[:,0],centers[:,1],centers[:,2])

# plt.show()

total_points = np.vstack([points,centers])
subdivs = []
for i,f in enumerate(faces):
	for k in range(len(f)):
		subdivs.append([f[k],f[(k+1)%len(f)],i+len(points)])
print(subdivs)


# ax = plt.axes(projection='3d')
# for f in subdivs:
# 	ax.plot3D(total_points[f,0],total_points[f,1],total_points[f,2])

# plt.show()

with open('subdiv.txt','w') as f:
	for p in total_points:
		f.write('v {} {} {}\n'.format(p[0],p[1],p[2]))

	f.write('\n')

	for s in subdivs:
		f.write('f {} {} {}\n'.format(s[0]+1,s[1]+1,s[2]+1))
