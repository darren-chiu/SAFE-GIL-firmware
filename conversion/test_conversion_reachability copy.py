# from gil_utils import MLP
# from gil_utils import load_model

from generate_safe_gil import generate_c_model

import os, re

import torch
import numpy as np
import torch.nn as nn





import torch
from reach_model_w_walls import dynamics
from reach_model_w_walls import modules




def get_opt_dstb(state, tMax, dstb_percentage, model,dynamics_):

  coords_tensor=torch.zeros(8)
  coords_tensor[0]=tMax
  coords_tensor[1:7]=state
  coords_tensor[7]=dstb_percentage

  coords_tensor[1:] = torch.clamp(coords_tensor[1:], torch.FloatTensor(dynamics_.state_test_range(
        ))[..., 0], torch.FloatTensor(dynamics_.state_test_range())[..., 1])
  coords_tensor.requires_grad_(True)
  results = model({'coords': dynamics_.coord_to_input(coords_tensor).unsqueeze(0)})
  print( results["model_out"])
  dvs = dynamics_.io_to_dv(
        results["model_in"], results["model_out"].squeeze(dim=-1)).detach()
  
  disturbance = dynamics_.optimal_disturbance(
        coords_tensor[1:], dvs[..., 1:])
  return disturbance.detach()

dynamics_=dynamics.Drone6DConditioned()
model = modules.SingleBVPNetEval(in_features=dynamics_.input_dim, out_features=1, type='sine', mode='mlp',
                                     final_layer_factor=1., hidden_features=64, num_hidden_layers=3)
model.load_state_dict(torch.load(
        "conversion/reach_model_w_walls/model_easier_sine.pth",map_location=torch.device('cpu') )["model"])
model.eval()




# # see the layers of the model
# for name, param in model.named_parameters():
#     print(name, param.shape)

# # check the wheights of the model
# for name, param in model.named_parameters():
#     print(name, param)


# #Feed model into script
# generate_c_model(model, "1obsreach_model_w_walls.c", "c_models/", testing=False)




tMax = 1.5

state=torch.tensor([0,0,0.5,1.5,0,0])

# state=torch.randn(6)
tMax = 1.4
dstb_percentage=0.45 # range: [0,0.5]
print(state,get_opt_dstb(state,tMax,dstb_percentage,model,dynamics_))



a = 1







