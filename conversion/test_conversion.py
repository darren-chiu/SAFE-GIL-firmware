from gil_utils import MLP
from gil_utils import load_model

from generate_safe_gil import generate_c_model

import os, re

import torch
import numpy as np
import torch.nn as nn



class MLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_layer_list, activation, drop_prob=0, use_batch_norm=False, verbose=False):
        super().__init__()
        
        self.input_fc = input_dim
        self.output_fc = output_dim
        self.hidden_layer_list = hidden_layer_list 
        self.activation = activation
        self.drop_prob = drop_prob
        self.use_batch_norm = use_batch_norm
        self.verbose = verbose
        self.use_drop_out = True if drop_prob > 0 else False
        assert not self.use_batch_norm, 'batch norm is not implemented for MLP'


        if self.activation.lower() == 'elu':
            activation_layer = nn.ELU()
        elif self.activation.lower() == 'sigmoid':    
            activation_layer = nn.Sigmoid()
        elif self.activation.lower() == 'relu':    
            activation_layer = nn.ReLU()
        elif self.activation.lower() == 'tanh':
            activation_layer = nn.Tanh()
        

        self.flatten = nn.Flatten()
        # fully connected layers
        mlp = []
        mlp.append(nn.Linear(self.input_fc, self.hidden_layer_list[0]))
        mlp.append(activation_layer)

        for hidden_layer_nbr in range( len(self.hidden_layer_list) - 1 ):
            mlp.append(nn.Linear(self.hidden_layer_list[hidden_layer_nbr], self.hidden_layer_list[hidden_layer_nbr + 1]))
            mlp.append(activation_layer)
            if self.use_drop_out:
                mlp.append(nn.Dropout(drop_prob))

        mlp.append(nn.Linear(self.hidden_layer_list[-1], self.output_fc))
        self.mlp = nn.Sequential(*mlp)

    # @torch.compile
    def forward(self, x):
        x = self.flatten(x)
        out = self.mlp(x)
        if self.verbose:
            print('Output shape is: {}'.format(list(out.shape)))
        return out



def get_latest_checkpoint(path_to_search, str_to_search='_b'):
    '''
    this function search for the last model saved that has str_to_search in it.
    '''
    def extract_number(f):
        s = re.findall("\\d+",f)
        return (int(s[0]) if s else -1,f)
    def find_files(str_to_search, path_to_search):
        results = []

        # Wlaking top-down from the root
        for root, dir, file_names in os.walk(path_to_search):
            for file_name in file_names:
                if str_to_search in file_name:
                    results.append(file_name)
        return results    

    list_of_files = find_files(str_to_search, path_to_search)
    try:
        checkpoint_to_get = max(list_of_files,key=extract_number)
    except:
        checkpoint_to_get = '11'
        print("An exception occurred")
    return path_to_search + '/' + checkpoint_to_get




# get these in iteration
config = {}
config['model_architecture'] = 'mlp'
config['input_dim'] = 23 # 23 with the sensor readings, wout 15
config['output_dim'] = 4
config['hidden_layer_list'] = [16, 16, 16]
config['activation'] = 'tanh'
config['drop_prob'] = 0.0
config['use_batch_norm'] = False
config['model_verbose'] = False
config['criterion_name'] = 'mae'
config['optimizer_name'] = 'adam'
config['lr'] = 0.001
config['l2_reg'] = 0.0001
config['device'] = 'cpu'
config['path_to_restore'] = 'conversion/[16,16,16]_tanh'

model = MLP(config['input_dim'], config['output_dim'], config['hidden_layer_list'], config['activation'], config['drop_prob'], config['use_batch_norm'], config['model_verbose'])


path_to_load=get_latest_checkpoint( config['path_to_restore'], str_to_search='_b')


checkpoint = torch.load(path_to_load, map_location=config['device'])

model.load_state_dict(checkpoint['model_state_dict'])

# see the layers of the model
for name, param in model.named_parameters():
    print(name, param.shape)

# check the wheights of the model
for name, param in model.named_parameters():
    print(name, param)

# Load Model
# TODO: Model dimensions?
#Don't think I need optimizer?

#Feed model into script
generate_c_model(model, "test_model.c", "c_models/", testing=False)
