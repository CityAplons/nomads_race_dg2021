import numpy as np

class PotentialField():
    """
    A class used to implement potential field repulsion controller.
    Класс имплементирует потенциальные поля для определения последующих столкновений.
    Attributes
    ----------
    drone_id : int
        Current CopterController id, needed to find self collision
        Текущий id контроллера. Необходим для избежания самоотталкивания 
    radius : float
        Potential field radius (meters)
        Радиус потенциального поля
    k_push : float
        Field gain,
        Коэффициент усиления поля
    Methods
    -------
    update(poses=None)
        Calculates momentum repulsion vector
        Вычисляет вектор отталкивания
    """
    def __init__(self, drone_id, radius, k_push):
        self.r = radius
        self.k = k_push
        
        # Array index to ignore
        self.ignore = drone_id-1

        self.primary_pose = np.array([0., 0., 0.])

    def update(self, poses=None):
        # Calculate distances from primary to each other drone
        self.primary_pose = poses[self.ignore]
        distances = self.calc_distances(poses)

        # Get list of drones and their poses that are located in pf sphere
        danger_poses = [ [pose, i] for i, pose in enumerate(poses) if distances[i] <= self.r ]

        # Resulting vector
        vec = np.array([0., 0., 0.])

        # For each pose in the sphere of current drone calculate cumulative repulsion vector
        for t in danger_poses:
            pose = t[0]
            drone_id = t[1]
            if(drone_id != self.ignore):
                # Regularization formula
                amplifier = self.k * (self.r - distances[drone_id])**2
                # Summing up vectors
                vec += self.vectorize(self.primary_pose, pose)*amplifier
        return vec

    def calc_distances(self, poses):
        """ Returns list of distances from primary to each other drone """
        dist = []
        for pose in poses:
            dist.append(self.distance(self.primary_pose, pose))
        return dist

    def distance(self, p1, p2):
        """ Calculates Euclidean distance between two points """
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 +(p1[2]-p2[2])**2)
    
    def vectorize(self, p1, p2):
        """ Making normalized vector from two points """
        delta = p1 - p2
        norm = self.distance(p1,p2)
        return delta/norm