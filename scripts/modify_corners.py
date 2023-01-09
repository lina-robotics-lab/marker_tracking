import pickle as pkl

import geometry_msgs.msg

fn = '../data/corners.pkl' or input('Please input the path to the .pkl file storing the corners[default:../data/corners.pkl]:')
with open(fn,'rb') as f:
    data = pkl.load(f)
    corners = data['corner_poses']

z_offset = -0.2
for i in range(len(corners)):
	corners[i].position.z+=z_offset

data['corner_poses'] = corners

with open(fn,'wb') as f:
    pkl.dump(data,f)