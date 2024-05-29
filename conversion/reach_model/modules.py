import torch
from torch import nn
import numpy as np
from collections import OrderedDict
import math
import torch.nn.functional as F


class BatchLinear(nn.Linear):
    '''A linear layer'''
    __doc__ = nn.Linear.__doc__

    def forward(self, input, params=None):
        if params is None:
            params = OrderedDict(self.named_parameters())

        bias = params.get('bias', None)
        weight = params['weight']

        output = input.matmul(weight.permute(
            *[i for i in range(len(weight.shape) - 2)], -1, -2))
        output += bias.unsqueeze(-2)
        return output


class Sine(nn.Module):
    def __init(self):
        super().__init__()

    def forward(self, input):
        # See paper sec. 3.2, final paragraph, and supplement Sec. 1.5 for discussion of factor 30
        # freqs = torch.linspace(
        #     3, 300, steps=input.shape[-1])[None, None, :].to(input.device)
        # return torch.sin(freqs * input)
        return torch.sin(30 * input)


class Saturation(nn.Module):
    def __init(self):
        super().__init__()

    def forward(self, input):
        # See paper sec. 3.2, final paragraph, and supplement Sec. 1.5 for discussion of factor 30
        return torch.minimum(torch.maximum(input, torch.zeros_like(input)), torch.ones_like(input))


class FCBlock(nn.Module):
    '''A fully connected neural network.
    '''

    def __init__(self, in_features, out_features, num_hidden_layers, hidden_features,
                 outermost_linear=False, nonlinearity='relu', weight_init=None):
        super().__init__()

        self.first_layer_init = None

        # Dictionary that maps nonlinearity name to the respective function, initialization, and, if applicable,
        # special first-layer initialization scheme
        nls_and_inits = {'sine': (Sine(), sine_init, first_layer_sine_init),
                         'relu': (nn.ReLU(inplace=True), init_weights_normal, None),
                         'sigmoid': (nn.Sigmoid(), init_weights_xavier, None),
                         'tanh': (nn.Tanh(), init_weights_xavier, None),
                         'selu': (nn.SELU(inplace=True), init_weights_selu, None),
                         'softplus': (nn.Softplus(), init_weights_normal, None),
                         'elu': (nn.ELU(inplace=True), init_weights_elu, None)}

        nl, nl_weight_init, first_layer_init = nls_and_inits[nonlinearity]

        if weight_init is not None:  # Overwrite weight init if passed
            self.weight_init = weight_init
        else:
            self.weight_init = nl_weight_init

        self.net = []
        self.net.append(nn.Sequential(
            BatchLinear(in_features, hidden_features), nl
        ))

        for i in range(num_hidden_layers):
            self.net.append(nn.Sequential(
                BatchLinear(hidden_features, hidden_features), nl
            ))

        if outermost_linear:
            self.net.append(nn.Sequential(
                BatchLinear(hidden_features, out_features)))
        else:
            self.net.append(nn.Sequential(
                BatchLinear(hidden_features, out_features), nl
            ))

        self.net = nn.Sequential(*self.net)
        if self.weight_init is not None:
            self.net.apply(self.weight_init)

        # Apply special initialization to first layer, if applicable.
        if first_layer_init is not None:
            self.net[0].apply(first_layer_init)

    def forward(self, coords, params=None, **kwargs):
        if params is None:
            params = OrderedDict(self.named_parameters())

        output = self.net(coords)
        return output
# class SingleBVPNet(nn.Module):
#     '''A canonical representation network for a BVP.'''

#     def __init__(self, out_features=1, type='sine', in_features=2,
#                  mode='mlp', hidden_features=256, num_hidden_layers=3, **kwargs):
#         super().__init__()
#         self.mode = mode
#         num_fourier_feature = int(hidden_features/2)
#         self.net = FCBlock(in_features=num_fourier_feature*2, out_features=out_features, num_hidden_layers=num_hidden_layers,
#                            hidden_features=hidden_features, outermost_linear=True, nonlinearity=type)
#         # self.fct = nn.Linear(1, 1)
#         self.B = torch.normal(mean=torch.zeros(
#             num_fourier_feature, in_features), std=1).cuda()
#         print(self)

#     def forward(self, model_input, params=None):
#         if params is None:
#             params = OrderedDict(self.named_parameters())

#         # Enables us to compute gradients w.r.t. coordinates
#         coords_org = model_input['coords'].clone(
#         ).detach().requires_grad_(True)

#         # obtain fourier embeddings
#         fourier_embeddinds = torch.cat((torch.sin(torch.matmul(self.B, coords_org.squeeze().T).T.unsqueeze(
#             0)), torch.cos(torch.matmul(self.B, coords_org.squeeze().T).T.unsqueeze(0))), dim=-1)

#         output = self.net(fourier_embeddinds)
#         return {'model_in': coords_org, 'model_out': output}


class SingleBVPNet(nn.Module):
    '''A canonical representation network for a BVP.'''

    def __init__(self, out_features=1, type='sine', in_features=2,
                 mode='mlp', hidden_features=256, num_hidden_layers=3, **kwargs):
        super().__init__()
        self.mode = mode

        self.net = FCBlock(in_features=in_features, out_features=out_features, num_hidden_layers=num_hidden_layers,
                           hidden_features=hidden_features, outermost_linear=True, nonlinearity=type)
        # self.fct = nn.Linear(1, 1)

        print(self)

    def forward(self, model_input, params=None):
        if params is None:
            params = OrderedDict(self.named_parameters())

        # Enables us to compute gradients w.r.t. coordinates
        coords_org = model_input['coords'].clone(
        ).detach().requires_grad_(True)

        output = self.net(coords_org)
        return {'model_in': coords_org, 'model_out': output}


class SingleBVPNetEval(nn.Module):
    '''A canonical representation network for a BVP.'''

    def __init__(self, out_features=1, type='sine', in_features=2,
                 mode='mlp', hidden_features=256, num_hidden_layers=3, **kwargs):
        super().__init__()
        self.mode = mode
        self.net = FCBlock(in_features=in_features, out_features=out_features, num_hidden_layers=num_hidden_layers,
                           hidden_features=hidden_features, outermost_linear=True, nonlinearity=type)
        # self.fct = nn.Linear(1, 1)
        print(self)

    def forward(self, model_input, params=None):
        if params is None:
            params = OrderedDict(self.named_parameters())

        output = self.net(model_input['coords'])
        return {'model_in': model_input['coords'], 'model_out': output}


########################
# Initialization methods
def init_weights_normal(m):
    if type(m) == BatchLinear or type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            nn.init.kaiming_normal_(
                m.weight, a=0.0, nonlinearity='relu', mode='fan_in')
            # m.weight.uniform_(0.0, 0.0)


def init_weights_selu(m):
    if type(m) == BatchLinear or type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            nn.init.normal_(m.weight, std=1 / math.sqrt(num_input))


def init_weights_elu(m):
    if type(m) == BatchLinear or type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            nn.init.normal_(m.weight, std=math.sqrt(
                1.5505188080679277) / math.sqrt(num_input))


def init_weights_xavier(m):
    if type(m) == BatchLinear or type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            nn.init.xavier_normal_(m.weight)


def sine_init(m):
    with torch.no_grad():
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            # See supplement Sec. 1.5 for discussion of factor 30
            m.weight.uniform_(-np.sqrt(6 / num_input) / 30,
                              np.sqrt(6 / num_input) / 30)


def first_layer_sine_init(m):
    with torch.no_grad():
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            # See paper sec. 3.2, final paragraph, and supplement Sec. 1.5 for discussion of factor 30
            m.weight.uniform_(-1 / num_input, 1 / num_input)
            # m.weight.uniform_(0.0, 0.0)
