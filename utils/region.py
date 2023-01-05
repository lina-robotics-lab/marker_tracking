import numpy as np
from scipy.spatial import ConvexHull
class AcquisitionRegion:
    def __init__(self,corner_coordinates,initital_corner = 0):
        '''
            corner_coordinates: shape = (n_corners,n_dim)
            initial_corner: the index indicating which corner is the initial location. By default it is corner 0.
        '''
        self.corners = corner_coordinates
        self.n_corners,self.n_dim =self.corners.shape
        
        self.hull = ConvexHull(self.corners)
        self.A = self.hull.equations[:,:-1]
        self.b = -self.hull.equations[:,-1].reshape(-1,1)
        # The point x within self.hull (shape = (n_dim,1)) satisfies self.A.dot(x)<=self.b
    def contains(self,test_pts):
        '''
            test_pts: shape = (n_dim, n_pts)
            Output: a boolean array indicating whether the test_pts are within self.hull
        '''
        return np.all(self.A.dot(test_pts)<=self.b,axis = 0)
    def grid(self,d):
        '''
            d: the distance between two grid points
            Output: a list of grid points within self.hull, with distance between two consecutive points being d. 
        '''

        cmax = np.max(self.corners,axis=0)

        cmin = np.min(self.corners,axis=0)
        xi = []
        for i in range(len(cmin)):
            xi.append(np.linspace(cmin[i],cmax[i],int((cmax[i]-cmin[i])/d)+1))

        x,y,z= np.meshgrid(*xi)
        x = x.flatten()
        y = y.flatten()
        z = z.flatten()

        grid = np.array(list(zip(x,y,z)))

        return grid[self.contains(grid.T)]
    
    def randomPos(self,N):
        '''
            N: the number of random positions to be sampled.
            Output: a list of N points sampled uniformly within self.full
        '''
        pos = []
        cmax = np.max(self.corners,axis=0)
        cmin = np.min(self.corners,axis=0)
        
        while len(pos)<=N:
            next_pos = np.random.uniform(cmax,cmin).reshape(-1,1)
            # print(self.contains(next_pos),next_pos)
            if self.contains(next_pos)[0]:
                pos.append(next_pos)
        return np.array(pos)