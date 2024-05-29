from gil_utils import MLP
from gil_utils import load_model

from generate_safe_gil import generate_c_model

import os, re

import torch
import numpy as np
import torch.nn as nn





import torch
from reach_model import dynamics
from reach_model import modules



dynamics_=dynamics.Drone6DConditioned()
model = modules.SingleBVPNetEval(in_features=dynamics_.input_dim, out_features=1, type='tanh', mode='mlp',
                                     final_layer_factor=1., hidden_features=64, num_hidden_layers=3)
model.load_state_dict(torch.load(
        "conversion/reach_model/model_tanh_easier.pth", map_location=torch.device('cpu') )["model"])
model.eval()
tMax = 1.4








# time, states, d percentage

# 1.4, m, m/s, 0.45


# see the layers of the model
for name, param in model.named_parameters():
    print(name, param.shape)

# check the wheights of the model
for name, param in model.named_parameters():
    print(name, param)


#Feed model into script
generate_c_model(model, "test_model.c", "c_models/", testing=False)
