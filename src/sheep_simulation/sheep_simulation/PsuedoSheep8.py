import random
import math
import numpy as np
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

class PsuedoSheep():
    ##Class Constants
    SEPARATION_DIST = 4.0
    BORDER_DIST=5.0
    ALIGNMENT_DIST = 9.0
    COHESSION_DIST = 10.0

    ##Weight modifieres (0.0 - 1.0)
    SEPARATION_WEIGHT = 2.0
    BORDER_WEIGHT=0.0
    ALIGNMENT_WEIGHT = 0.1
    COHESSION_WEIGHT = 0.2

    ##Predefined Borders
    BORDER_RANGE = [(0,0), (60,60)]

    ##Movement
    UPDATE_INTERVAL = 1.0 #seconds 
    STEP = 1 ## Distance moved

    ##Class Variables
    sheep_pos = np.empty((0,3))

    @staticmethod
    # def generate_unique_coord(existing_coord):
    #     ##We don't care if th matches
    #     th_range = math.pi * 2
    #     th = np.random.uniform(th_range)

    #     ##loop until we generate valid coord
    #     coord_range = (20, 40)
    #     while True:
    #         x = np.random.uniform(*coord_range)
    #         y = np.random.uniform(*coord_range)
    #         candidate = np.array([x, y, 0])

    #         if existing_coord.size == 0:
    #             candidate[2] = th
    #             return candidate

    #         ##find the differences and compare
    #         differences = np.abs(existing_coord[:, :2] - candidate[:2])         
            
    #         differences = np.sum(differences ** 2, axis=1) ##calc euclidain

    #         if not any(differences[:] < PsuedoSheep.SEPARATION_DIST):
    #             candidate[2] = th
    #             return candidate
    def generate_unique_coord(existing_coord):
        ##We don't care if th matches
        th_range = math.pi * 2

        ##loop until we generate valid coord
        coord_range = (20, 40)
        while True:
            x = np.random.uniform(*coord_range)
            y = np.random.uniform(*coord_range)
            th=np.random.uniform(0,th_range)
            candidate = np.array([x, y, th])

            if existing_coord.size == 0:
                return candidate

            ##find the differences and compare
            differences = np.abs(existing_coord[:, :2] - candidate[:2])         
            
            differences = np.sum(differences ** 2, axis=1) ##calc euclidain

            if not any(differences < PsuedoSheep.SEPARATION_DIST):
                return candidate

    @staticmethod
    def generate_neighbours(candidate, other_coord, distance):
        ##find the differences between all coords
        differences = np.abs(other_coord - candidate).astype(float)

         # Mask for valid neighbors: within distance and not zero distance
        squared_distances = (differences[:, 0] ** 2) + (differences[:, 1] ** 2)
        mask = (squared_distances <= distance ** 2) & ((squared_distances > 0) & (differences[:, 2] > 0))
        
        filtered_array = other_coord[mask]
        return filtered_array

    @staticmethod
    def generate_Borders(candidate, other_coord, distance):
        ##find the differences between all coords
        differences = np.abs(other_coord - candidate).astype(float)

         # Mask for valid neighbors: within distance and not zero distance
        squared_distances = (differences[:, 0] ** 2) + (differences[:, 1] ** 2)
        mask = (squared_distances <= distance ** 2) & ((squared_distances > 0) & (differences[:, 2] > 0))
        
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
        bor_vector = self.calc_border()

        vector_array=np.array([self.calc_cohesion(),self.calc_seperation(),self.calc_alignment()])
        #vector_array=np.array([self.calc_seperation()])

        vector_array = vector_array[~np.all(vector_array == [0, 0], axis=1)]

        ##sum of 3 coordinates
        if len(vector_array)>0:
            final_vector=np.sum(vector_array,axis=0)/len(vector_array)
        else:
            final_vector=[0,0]
        if np.linalg.norm(final_vector)>0:
            final_vector=final_vector/np.linalg.norm(final_vector)
        # final_vector = np.sum(vector_array,axis=0)/len(vector_array)
        # #difference_vector = (final_vector - np.array(self.coord[:2]))
        # #final_angle = np.arctan2(difference_vector[0], difference_vector[1])
        final_angle = np.arctan2(final_vector[1], final_vector[0])

        final_angle = final_angle % (2 * np.pi)

        return (final_vector,final_angle)
    
    def calc_cohesion(self):
        ##Cohession
        ##Find all neighbours within coh radius
        #nbs_coh = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:,:2], PsuedoSheep.COHESSION_DIST)
        nbs_coh = PsuedoSheep.generate_neighbours(self.coord, PsuedoSheep.sheep_pos, PsuedoSheep.COHESSION_DIST)
        nbs_coh_len = len(nbs_coh)


        if nbs_coh_len > 0:
            ##Find the center between all coordinates within range
            center = nbs_coh.mean(axis = 0)
            coh_vector = (center[:2] - self.coord[:2]) * PsuedoSheep.COHESSION_WEIGHT

        else:
            coh_vector = np.array([0, 0])
        


        return coh_vector
    
    def calc_seperation(self):
        ##Seperation
        # nbs_sep = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:, :2], PsuedoSheep.SEPARATION_DIST)
        nbs_sep = PsuedoSheep.generate_neighbours(self.coord, PsuedoSheep.sheep_pos, PsuedoSheep.SEPARATION_DIST)
        nbs_sep_len = len(nbs_sep)


        ##Calcualting eculdiain
        if nbs_sep_len > 0:
            # sep_distances = nbs_sep[:,:2] - self.coord[:2]
            # sep_dist_square = np.sum(sep_distances ** 2, axis=1) ##calc euclidain

            # ##Calcualting inverse
            # sep_dist_square = np.where(sep_dist_square != 0, sep_dist_square, np.inf)
            # sep_dist_inv = np.where(sep_dist_square != 0, (1/sep_dist_square), 0)
            sep_distances = nbs_sep[:,:2] - self.coord[:2]
            sep_dist_square = np.sum(sep_distances ** 2, axis=1) ##calc euclidain

            ##Calcualting inverse
            sep_dist_inv = np.where(sep_dist_square != 0, (1/sep_dist_square), 0)

            

            ##Ajust the weight for each coordinate (based on how close they are)
            ##Sum the coordinates to find out how much we should transform starting coordinate
            ##Subtract and transform starting coordinate (find where we should go!)
            temp=sep_dist_inv[:, None]
            sum=np.sum(sep_distances * sep_dist_inv[:, None])
            sep_vector = (np.sum(sep_distances * sep_dist_inv[:, None], axis=0))*PsuedoSheep.SEPARATION_WEIGHT* -1
            print()

        else:
            sep_vector = np.array([0,0])
        
        if np.linalg.norm(sep_vector)>0:
            sep_vector=sep_vector/np.linalg.norm(sep_vector)

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
            #avg_th = np.arctan2(mean_cos, mean_sin)

            ali_vector = np.array([np.cos(avg_th), np.sin(avg_th)]) * PsuedoSheep.ALIGNMENT_WEIGHT

        else:
            ali_vector = np.array([0,0])
        


        return ali_vector
    
    def calc_border(self):
        ##Seperation
        # nbs_sep = PsuedoSheep.generate_neighbours(self.coord[:2], PsuedoSheep.sheep_pos[:, :2], PsuedoSheep.SEPARATION_DIST)
        nbs_sep = PsuedoSheep.generate_neighbours(self.coord, PsuedoSheep.sheep_pos, PsuedoSheep.SEPARATION_DIST)
        nbs_sep_len = len(nbs_sep)


        ##Calcualting eculdiain
        if nbs_sep_len > 0:
            sep_distances = nbs_sep[:,:2] - self.coord[:2]
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
        

        if np.linalg.norm(sep_vector)>0:
            sep_vector=sep_vector/np.linalg.norm(sep_vector)

        return sep_vector
    
    

    def update_position(self):
        values=self.update_velocity()
        #random_noise=np.random.uniform(-0.1,0.1)
        random_noise=0
        th = values[1]
        dx = (values[0][0]+random_noise)
        dy = (values[0][1]+random_noise)

        step = np.array([dx, dy,th])
        if np.linalg.norm(step)>0:
            step=step/np.linalg.norm(step)
        ##step_normalized = step / np.linalg.norm(step)  # This ensures step size = 1

        self.coord += step


        PsuedoSheep.sheep_pos[self.index] = self.coord



def animate(frame, ax1, sheep_list):
    for sheep in sheep_list:
        sheep.update_position()

    ax1.clear()
    ax1.scatter(PsuedoSheep.sheep_pos[:, 0], PsuedoSheep.sheep_pos[:, 1], label="Sheep", color="blue")

    coords_transformed = np.column_stack([(np.sin(PsuedoSheep.sheep_pos[:, 0]) * 2), (np.cos(PsuedoSheep.sheep_pos[:, 1]) * 2)])

    display_points = np.column_stack([PsuedoSheep.sheep_pos[:, 0], PsuedoSheep.sheep_pos[:, 1], coords_transformed[:, 0], coords_transformed[:, 1]])

    for points in display_points:
        ax1.quiver(points[0], points[1], points[2], points[3], angles='xy', scale_units='xy', scale=1, color="red", width=0.003)

    plt.xlim(0, 60)
    plt.ylim(0, 60)

def main():
    print("Hello World")

    style.use('fivethirtyeight')
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    # sheep1 = PsuedoSheep()
    # sheep2 = PsuedoSheep()
    # sheep3 = PsuedoSheep()
    # sheep4 = PsuedoSheep()
    # sheep5 = PsuedoSheep()
    # sheep6 = PsuedoSheep()
    # sheep7 = PsuedoSheep()
    # sheep8 = PsuedoSheep()

    # sheep_list = [sheep1, sheep2, sheep3, sheep4, sheep5, sheep6, sheep7, sheep8]

    sheep_list=[PsuedoSheep() for i in range(0,random.randint(20,30))]
    print(len(sheep_list))

    # Start the animation
    ani = animation.FuncAnimation(fig, animate, fargs=(ax1, sheep_list), interval=50)
    plt.show()

"""     PsuedoSheep.sheep_pos = np.stack([[40.5, 39.5, 3.9], [40, 40.5, 3.9], [40, 40, 3.9], [40, 41, 3.9]])
    sheep1.coord = np.array(PsuedoSheep.sheep_pos[0])
    sheep2.coord = np.array(PsuedoSheep.sheep_pos[1])
    sheep3.coord = np.array(PsuedoSheep.sheep_pos[2])
    sheep4.coord = np.array(PsuedoSheep.sheep_pos[3]) """

if __name__ == '__main__':
    main()
