import pickle as pkl
import numpy as np

with open('subdived2.txt') as f:
	lines = np.array([[float(s) for s in l.strip().split(' ')] for l in f.readlines()])



with open('../data/side_faces2.pkl','wb') as f:
	pkl.dump(np.array(lines),f)

