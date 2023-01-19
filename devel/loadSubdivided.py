import pickle as pkl
import numpy as np

from geometry_msgs.msg import PoseStamped

with open('subdived.txt') as f:
	lines = np.array([[float(s) for s in l.strip().split(' ')] for l in f.readlines()])

side_faces = []
for l in lines:
	p = PoseStamped()
	p.pose.position.x = l[0]
	p.pose.position.y = l[1]
	p.pose.position.z = l[2]
	side_faces.
with open('../data/side_faces.pkl','wb') as f:
	pkl.dump(lines,f)

