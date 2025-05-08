

import numpy as np

data = np.load("003_cracker_box.npz", allow_pickle=True)

top_30_grasps = data['top_30_grasps_dict'].item()
# print(top_30_grasps)

top_30_grasps = data['top_30_scores_dict'].item()
print(top_30_grasps)

# top_30_grasps= top_30_grasps[255][::-1] 
# print(top_30_grasps)

