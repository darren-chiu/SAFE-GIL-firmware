
import torch
from torch import nn
import numpy as np



class MLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_layer_list, activation, drop_prob=0, use_batch_norm=False, verbose=True):
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
        elif self.activation.lower() == 'sine':
            activation_layer = Sine()
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


        model = MLP( 23, 4, self.config['hidden_layer_list'], self.config['activation'], 0, False, self.config['model_verbose'])




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

    path_to_load=get_latest_checkpoint( self.config['path_to_restore'], str_to_search='_b')



# function to load the model and optimizer states        
def load_model( model, optimizer, path_to_load, device='cpu'):

   checkpoint = torch.load(path_to_load, map_location=device)
   model.load_state_dict(checkpoint['model_state_dict'])
#    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
   epoch_n = checkpoint['epoch_n']

   print(f'model is being loaded from previously trained checkpoint (epoch{epoch_n}) from {path_to_load}')
   return model
#    return model, optimizer, epoch_n



# these values are used for converting the model output to the actual thrust pwm values
expert_actions_mean = torch.tensor([48054.22234259, 46984.33434234, 50876.45975656, 42471.35183533])
expert_actions_std = torch.tensor([3781.97827489, 4069.80051149, 3844.66114359, 4089.51596848])


# this is how I process the states before feeding them to the model
# processed_states = visited_states.copy()
# processed_states[:,2] = processed_states[:,2] - 0.4
# processed_states[:,6] = processed_states[:,6] - 0.8364401720725966
# processed_states[:,8] = processed_states[:,8] - 0.03843474001030632 
# processed_states[:,9:12] = (processed_states[:,9:12] - torch.tensor([-0.28475793, -0.17994721, -0.7365864 ]) ) / torch.tensor([25.70296269, 22.50442717,  1.83997706])

# how I standardize the expert actions
# standardized_expert_actions = (expert_actions - torch.tensor([48054.22234259, 46984.33434234, 50876.45975656, 42471.35183533])) / torch.tensor([3781.97827489, 4069.80051149, 3844.66114359, 4089.51596848])

def get_action(self, state_tensor, device ):
    if type(state_tensor) == np.ndarray:
        state_tensor = torch.tensor(state_tensor, dtype=torch.float32)
    self.model.eval()
    with torch.no_grad():
        # on top of the processing of the state tensor, I also convert the angles to sin and cos
        state_tensor = torch.cat( [ (state_tensor[0]  ).reshape(1), \
                        (state_tensor[1] ).reshape(1), \
                        (state_tensor[2] ).reshape(1), \
                        torch.sin(torch.deg2rad(state_tensor[3])).reshape(1), torch.cos(torch.deg2rad(state_tensor[3])).reshape(1), \
                        torch.sin(torch.deg2rad(state_tensor[4])).reshape(1), torch.cos(torch.deg2rad(state_tensor[4])).reshape(1), \
                        torch.sin(torch.deg2rad(state_tensor[5])).reshape(1), torch.cos(torch.deg2rad(state_tensor[5])).reshape(1), \
                        ((state_tensor[6] )).reshape(1), \
                        ((state_tensor[7] )).reshape(1), \
                        ((state_tensor[8] )).reshape(1), \
                        ((state_tensor[9] )).reshape(1), \
                        ((state_tensor[10])).reshape(1), \
                        ((state_tensor[11])).reshape(1), \
                        ((state_tensor[12])).reshape(1), \
                        ((state_tensor[13])).reshape(1), \
                        ((state_tensor[14])).reshape(1), \
                        ((state_tensor[15])).reshape(1), \
                        ((state_tensor[16])).reshape(1), \
                        ((state_tensor[17])).reshape(1), \
                        ((state_tensor[18])).reshape(1), \
                        ((state_tensor[19])).reshape(1) ])
        
        state_tensor = state_tensor.reshape((1, 23))
        state_tensor = state_tensor.to(device)
        action  = self.model( state_tensor )

        action = action.squeeze().detach()
        # converting the model output to the actual thrust pwm values
        action_unnormalized = action * self.expert_actions_std + self.expert_actions_mean 

    return action_unnormalized