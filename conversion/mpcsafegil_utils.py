
import torch
from torch import nn
import numpy as np



class LidarVelModel(nn.Module):
    def __init__(self):
        super(LidarVelModel, self).__init__()

        # multi-layer perceptron
        self.pi_1 = nn.Linear(10, 64)
        self.pi_2 = nn.Linear(64, 64)
        self.pi_3 = nn.Linear(64, 64)
        self.pi_4 = nn.Linear(64, 2)

        # self.loss_func = nn.MSELoss()

    # policy
    def forward(self, state):
        state[2:] = torch.round(state[2:], decimals=2)
        state = state - torch.tensor([0.0, 0.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]).to(state.device)
        state = state / torch.tensor([1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]).to(state.device)
        x = torch.tanh(self.pi_1(state))
        x = torch.tanh(self.pi_2(x))
        x = torch.tanh(self.pi_3(x))
        x = self.pi_4(x)
        x = x * torch.tensor([ roll_max, pitch_max]).to(state.device)
        return x
    

    def get_action(self, state_tensor, device ):
        if type(state_tensor) == np.ndarray:
            state_tensor = torch.tensor(state_tensor, dtype=torch.float32)
        self.eval()
        with torch.no_grad():
            state_tensor = state_tensor.to(device)
            action  = self.forward( state_tensor )
        return action.squeeze().detach()

# im_model = LidarVelModel()
# im_model.load_state_dict(torch.load( model_path ))


