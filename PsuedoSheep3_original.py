import random
import math
import numpy as np

class PsuedoSheep():
    ##Class Constants
    SEPARATION_DIST = 1.0
    ALIGNMENT_DIST = 5.0
    COHESSION_DIST = 7.0

    ##Weight modifieres (0.0 - 1.0)
    SEPARATION_WEIGHT = 1.0
    ALIGNMENT_WEIGHT = 1.0
    COHESSION_WEIGHT = 1.0

    ##Class Variables
    sheep_pos = np.empty((0,3))
    ##sheep_vel = np.array([])

    @staticmethod
    def generate_unique_coord(existing_coord):
        ##We don't care if th matches
        th_range = math.pi * 2
        th = np.random.uniform(th_range)

        ##loop until we generate valid coord
        coord_range = (0, 50)
        while True:
            x = np.random.uniform(*coord_range)
            y = np.random.uniform(*coord_range)
            candidate = np.array([x, y, 0])

            if existing_coord.size == 0:
                candidate[2] = th
                return candidate

            ##find the differences and compare
            differences = np.abs(existing_coord[:, :2] - candidate[:2])         
            
            differences = np.sum(differences ** 2, axis=1) ##calc euclidain

            if not any(differences[:] < PsuedoSheep.SEPARATION_DIST):
                candidate[2] = th
                return candidate
        

    @staticmethod
    def generate_neighbours(candidate, other_coord, distance):
        ##find the differences between all coords
        differences = np.abs(other_coord - candidate) 

         # Mask for valid neighbors: within distance and not zero distance
        squared_distances = (differences[:, 0] ** 2) + (differences[:, 1] ** 2)
        mask = (squared_distances <= distance ** 2) & (squared_distances > 0)
        
        filtered_array = other_coord[mask]
        return filtered_array



    def __init__(self):
        self.coord = PsuedoSheep.generate_unique_coord(PsuedoSheep.sheep_pos)
        
        self.index = len(PsuedoSheep.sheep_pos)


        PsuedoSheep.sheep_pos = np.vstack([PsuedoSheep.sheep_pos, self.coord]) ##Doesn't flatten the list

    def update_velocity(self):
        """ Implementing boid's algorithm - based on average of 3 vectors
        
            1. Seperation - Head away from boids within seperation radius - Inverse weighting
            2. Alignment - Slight change to average orientation 
            3. Cohession - Slighty head towards local flock centre"""


        """ Find average angle, since angles are cylindirical (e.g. 0 & 360) we can't just add, need to use sin & cos, then convert back into coord with arctan2 """

        #### Combining Vectors
        coh_vector = self.calc_cohesion()
        sep_vector = self.calc_seperation()
        ali_vector = self.calc_alignment()

        ##sum of 3 coordinates
        final_vector = (coh_vector + sep_vector + ali_vector) / 3
        difference_vector = (final_vector - np.array(self.coord[:2]))
        print(coh_vector, sep_vector, ali_vector,final_vector, difference_vector)
        final_angle = np.arctan2(difference_vector[1], difference_vector[0])

        return final_angle

    
    def calc_cohesion(self):
        ##Cohession
        ##Find all neighbours within coh radius
        nbs_coh = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:,:2], PsuedoSheep.COHESSION_DIST)
        nbs_coh_len = len(nbs_coh)


        if nbs_coh_len > 0:
            ##Find the center between all coordinates within range
            center = nbs_coh.mean(axis = 0)
            coh_vector = (center - self.coord[:2]) * PsuedoSheep.COHESSION_WEIGHT

        else:
            coh_vector = np.array([0, 0])
        
        return coh_vector
    
    def calc_seperation(self):
        ##Seperation
        nbs_sep = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:, :2], PsuedoSheep.SEPARATION_DIST)
        nbs_sep_len = len(nbs_sep)


        ##Calcualting eculdiain
        if nbs_sep_len > 0:
            sep_distances = nbs_sep - self.coord[:2]
            sep_dist_square = np.sum(sep_distances ** 2, axis=1) ##calc euclidain

            ##Calcualting inverse
            sep_dist_square = np.where(sep_dist_square != 0, sep_dist_square, np.inf)
            sep_dist_inv = np.where(sep_dist_square != 0, (1/sep_dist_square), 0)

            ##Ajust the weight for each coordinate (based on how close they are)
            ##Sum the coordinates to find out how much we should transform starting coordinate
            ##Subtract and transform starting coordinate (find where we should go!)
            sep_vector = self.coord[:2] - np.sum(sep_distances * sep_dist_inv[:, None], axis=0)

        else:
            sep_vector = np.array([0,0])
        

        return sep_vector
    
        """         ##Seperation - Old Code
        nbs_sep = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:, :2], PsuedoSheep.COHESSION_DIST)
        sep_vector = np.array([0,0])

        ##Give an inverse weighting adjusted to not be affected by different coord sizes (n/n^2)
        sep_weights = np.where(nbs_sep != 0, (nbs_sep ** 2), np.inf)
        sep_weights = np.where(nbs_sep != 0, (nbs_sep/sep_weights), 0)
        
        sep_vector = (sep_vector - nbs_sep.sum(axis = 0)) * PsuedoSheep.SEPARATION_WEIGHT


        ##Alignment
        nbs_ali = PsuedoSheep.generate_neighbours(self.coord, PsuedoSheep.sheep_pos, PsuedoSheep.ALIGNMENT_DIST) """
        
    def calc_alignment(self):

        ##Alignment
        nbs_ali = PsuedoSheep.generate_neighbours(self.coord, PsuedoSheep.sheep_pos, PsuedoSheep.ALIGNMENT_DIST)
        nbs_ali = nbs_ali[:, 2]
        nbs_ali_len = len(nbs_ali)

        ##Required as general mean will handle angles close to boundry poorly 
        if nbs_ali_len > 0:
            mean_sin = np.sin(nbs_ali).mean()
            mean_cos = np.cos(nbs_ali).mean()
            avg_th = np.arctan2(mean_sin, mean_cos)

            ali_vector = np.array([np.cos(avg_th), np.sin(avg_th)]) * PsuedoSheep.ALIGNMENT_WEIGHT

        else:
            ali_vector = np.array([0,0])
        

        return ali_vector


def main():
    print("Hello World")
    sheep1 = PsuedoSheep()
    sheep2 = PsuedoSheep()
    sheep3 = PsuedoSheep()
    sheep4 = PsuedoSheep()

    PsuedoSheep.sheep_pos = np.stack([[40.5, 39.5, 3.9], [40, 40.5, 3.9], [40, 40, 3.9], [40, 41, 3.9]])
    sheep1.coord = np.array(PsuedoSheep.sheep_pos[0])
    sheep2.coord = np.array(PsuedoSheep.sheep_pos[1])
    sheep3.coord = np.array(PsuedoSheep.sheep_pos[2])
    sheep4.coord = np.array(PsuedoSheep.sheep_pos[3])

    print(sheep3.update_velocity())
if __name__ == '__main__':
    main()
