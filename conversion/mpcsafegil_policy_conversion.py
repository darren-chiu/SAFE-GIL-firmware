from mpcsafegil_utils import LidarVelModel

from generate_safe_gil import generate_c_model

import os, re

import torch
import numpy as np
import torch.nn as nn






safegil_policy_im_model_path = 'conversion/expert_safegil_0.001lr_1000epoch/100dems/0/im_model.pt'
bc_policy_im_model_path = 'conversion/expert_0.001lr_1000epoch/100dems/0/im_model.pt'

method = 'bc' # 'bc' or 'mpcsafegil'

model_save_name = 'mpcsafegil_policy_100dems' if method == 'mpcsafegil' else 'mpcsafegil_bc_policy_100dems'
model_path = safegil_policy_im_model_path if method == 'mpcsafegil' else bc_policy_im_model_path

im_model = LidarVelModel()
im_model.load_state_dict(torch.load( model_path ))


# see the layers of the model
for name, param in im_model.named_parameters():
    print(name, param.shape)

# check the wheights of the model
for name, param in im_model.named_parameters():
    print(name, param)


#Feed model into script
generate_c_model(im_model, f"{model_save_name}.c", "c_models/", testing=False)

a = 1
