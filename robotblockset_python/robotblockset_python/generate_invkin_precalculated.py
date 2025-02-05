from itertools import product
from multiprocessing import Pool, Process
import numpy as np
from tqdm import tqdm
from robotblockset_python.robot_models.robot_spec import * #panda
from robotblockset_python.robot_models.robot_models import * #kinmodel_panda 

class Generate_invkin:
    def __init__(self, robot_name = 'panda', linspace_n_elements=14, save_folder = 'saved'):
        if robot_name == 'panda':
            robot_obj = panda()
            self.direct_kinematics_function = kinmodel_panda #robot_obj.kinmodel
            q_min = robot_obj.q_min
            q_max = robot_obj.q_max
        else:
            #add calc for other robot models. 
            raise Exception("Not implemented for robot model {}".format(robot_name))
        # Must be defined : q_min, q_max, linspace_n_elements
        query_qs = []    
        # Nej nardi to en vektorsko kdor zna :D
        for i in range(0,robot_obj.nj):
            query_q_joint_i = np.linspace(q_min[i], q_max[i], linspace_n_elements)
            query_qs.append(query_q_joint_i)
        print("Finished making q lin")
        #query_qs = np.array(query_qs)
        # Now generate queries, concrete values of q configurations based on the defined linspaces
        #out = np.array(np.meshgrid(query_qs))[0,:]
        sampled_q_combinations = np.array(list(product(*query_qs)))
        print(sampled_q_combinations.shape)
        direct_kinematics = np.zeros((sampled_q_combinations.shape[0], robot_obj.nj))
        n_cores = 6
        split_inputs = np.array_split(sampled_q_combinations, n_cores, axis=0)
        print("Starting multiprocessing")
            
        pool = Pool(processes=n_cores)
        direct_kinematics = pool.map(self.solve_direct_kinematics, split_inputs)
        pool.close()
        pool.join()

         
        direct_kinematics = np.vstack(direct_kinematics)
        #direct_kinematics = 
        print("DIR", direct_kinematics.shape)
        print("SMP", sampled_q_combinations.shape)

        #return None
        concatenated_q_and_x = np.hstack([direct_kinematics, sampled_q_combinations])   
        fn_to_save = '{}/concatenated_q_and_x_linsp_{}.npy'.format(save_folder, str(linspace_n_elements))
        np.save(fn_to_save, concatenated_q_and_x)
        print("Saved q and x array to {}".format(fn_to_save))
        return None

    def solve_direct_kinematics(self, qs):
        out = np.zeros((qs.shape[0], 7)) # 7 for 3p+4quat
        for i in range(qs.shape[0]):
            out[i,:], _ = self.direct_kinematics_function(q=qs[i,:],tcp=None,out='x')

        return out

if __name__=='__main__':
    Generate_invkin()
