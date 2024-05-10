from gil_utils import MLP
from gil_utils import load_model

from generate_safe_gil import generate_c_model

import torch
import numpy as np
import torch.nn as nn

# Load Model
# TODO: Model dimensions?
#Don't think I need optimizer?
model = load_model(MLP(), None, "model_1500")

#Feed model into script
generate_c_model(model, "test_model.c", "c_models/", testing=False)