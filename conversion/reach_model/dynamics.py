from abc import ABC, abstractmethod
# from utils import diff_operators, quaternion

import math
import torch

# during training, states will be sampled uniformly by each state dimension from the model-unit -1 to 1 range (for training stability),
# which may or may not correspond to proper test ranges
# note that coord refers to [time, *state], and input refers to whatever is fed directly to the model (often [time, *state, params])
# in the future, code will need to be fixed to correctly handle parametrized models


class Dynamics(ABC):
    def __init__(self, name: str,
                 loss_type: str, set_mode: str,
                 state_dim: int, input_dim: int,
                 control_dim: int, disturbance_dim: int,
                 state_mean: list, state_var: list,
                 value_mean: float, value_var: float, value_normto: float,
                 deepReach_model: bool, exact_factor: float):
        self.name = name
        self.loss_type = loss_type
        self.set_mode = set_mode
        self.state_dim = state_dim
        self.input_dim = input_dim
        self.control_dim = control_dim
        self.disturbance_dim = disturbance_dim
        self.state_mean = torch.tensor(state_mean)
        self.state_var = torch.tensor(state_var)
        self.value_mean = value_mean
        self.value_var = value_var
        self.value_normto = value_normto
        self.deepReach_model = deepReach_model
        self.exact_factor = exact_factor

        assert self.loss_type in [
            'brt_hjivi', 'brat_hjivi'], f'loss type {self.loss_type} not recognized'
        if self.loss_type == 'brat_hjivi':
            assert callable(self.reach_fn) and callable(self.avoid_fn)
        assert self.set_mode in [
            'reach', 'avoid'], f'set mode {self.set_mode} not recognized'
        for state_descriptor in [self.state_mean, self.state_var]:
            assert len(state_descriptor) == self.state_dim, 'state descriptor dimension does not equal state dimension, ' + \
                str(len(state_descriptor)) + ' != ' + str(self.state_dim)

    # ALL METHODS ARE BATCH COMPATIBLE

    # MODEL-UNIT CONVERSIONS (TODO: refactor into separate model-unit conversion class?)

    # convert model input to real coord
    def input_to_coord(self, input):
        coord = input.clone()
        coord[..., 1:] = (input[..., 1:] * self.state_var.to(device=input.device)
                          ) + self.state_mean.to(device=input.device)
        return coord

    # convert real coord to model input
    def coord_to_input(self, coord):
        input = coord.clone()
        input[..., 1:] = (coord[..., 1:] - self.state_mean.to(device=coord.device)
                          ) / self.state_var.to(device=coord.device)
        return input

    # convert model io to real value
    def io_to_value(self, input, output):
        if self.deepReach_model == 'diff':
            return (output * self.value_var / self.value_normto) + self.boundary_fn(self.input_to_coord(input)[..., 1:])
        elif self.deepReach_model == 'exact':

            # return (output * input[..., 0] * self.value_var / self.value_normto) + self.boundary_fn(self.input_to_coord(input)[..., 1:])

            # Crude Fix to handle NaN in Rocket Landing. V(x,t) = l(x) + (k*t + (1-k))*NN(x,t).
            # k=1 gives us V(x,t) = l(x) + t*NN(x,t) which is the correct exact_BC variant.
            # Setting k=1 gives NaN for the Rocket Landing example as the PDE loss but works if we keep k = 1 - ε where ε <<<< 1.
            # Hence, we keep ε = 1e-7. I feel the performance won't vary much as k ≈ 1.
            # Finally, k=1 for every variant except Rocket Landing where k = 0.9999999.
            k = self.exact_factor
            exact_BC_factor = k * input[..., 0] + (1-k)
            return (output * exact_BC_factor * self.value_var / self.value_normto) + self.boundary_fn(self.input_to_coord(input)[..., 1:])
        elif self.deepReach_model == 'exact_sin':
            # just for testing purpose, tMax is hardcoded to be 2
            return (output * torch.sin(input[..., 0]/2) * self.value_var / self.value_normto) + self.boundary_fn(self.input_to_coord(input)[..., 1:])
        elif self.deepReach_model == 'exact_exp':
            return (output * (-torch.exp(-5*input[..., 0])+1.0) * self.value_var / self.value_normto) + self.boundary_fn(self.input_to_coord(input)[..., 1:])
        elif self.deepReach_model == 'exact_diff':
            # V(x,t)= l(x) + NN(x,t) - NN(x,0)
            # print(input.shape, output.shape)
            # print(output[0])
            # print(output[1])
            output0 = output[0].squeeze(dim=-1)
            output1 = output[1].squeeze(dim=-1)
            return (output0 - output1) * self.value_var / self.value_normto + self.boundary_fn(self.input_to_coord(input[0].detach())[..., 1:])
        elif self.deepReach_model == 'reg':
            return (output * self.value_var / self.value_normto) + self.value_mean
        else:
            raise NotImplementedError

    # convert model io to real dv
    def io_to_dv(self, input, output):
        if self.deepReach_model == 'exact_diff':

            dodi1 = diff_operators.jacobian(
                output[0], input[0])[0].squeeze(dim=-2)
            dodi2 = diff_operators.jacobian(
                output[1], input[1])[0].squeeze(dim=-2)

            dvdt = (self.value_var / self.value_normto) * dodi1[..., 0]

            dvds_term1 = (self.value_var / self.value_normto /
                          self.state_var.to(device=dodi1.device)) * (dodi1[..., 1:]-dodi2[..., 1:])

            state = self.input_to_coord(input[0])[..., 1:]
            dvds_term2 = diff_operators.jacobian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            dvds = dvds_term1 + dvds_term2
            return torch.cat((dvdt.unsqueeze(dim=-1), dvds), dim=-1)

        dodi = diff_operators.jacobian(
            output.unsqueeze(dim=-1), input)[0].squeeze(dim=-2)

        if self.deepReach_model == 'diff':
            dvdt = (self.value_var / self.value_normto) * dodi[..., 0]

            dvds_term1 = (self.value_var / self.value_normto /
                          self.state_var.to(device=dodi.device)) * dodi[..., 1:]
            state = self.input_to_coord(input)[..., 1:]
            dvds_term2 = diff_operators.jacobian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            dvds = dvds_term1 + dvds_term2

        elif self.deepReach_model == 'exact':

            # Shrewd Fix to handle NaN in Rocket Landing. V(x,t) = l(x) + (k*t + (1-k))*NN(x,t).
            # k=1 gives us V(x,t) = l(x) + t*NN(x,t) which is the original variant.
            # Setting k=1 gives NaN as the PDE loss but works if we keep k = 1 - ε where ε <<<< 1.
            # Hence, we keep ε = 1e-7. I feel the performance won't vary much as k ≈ 1.
            # To summarize, k=1 for every variant except Rocket Landing where k = 0.9999999.
            k = self.exact_factor
            exact_BC_factor = k * input[..., 0] + (1-k)
            exact_factor_der = k
            dvdt = (self.value_var / self.value_normto) * \
                (exact_BC_factor*dodi[..., 0] + exact_factor_der*output)

            dvds_term1 = (self.value_var / self.value_normto /
                          self.state_var.to(device=dodi.device)) * dodi[..., 1:] * exact_BC_factor.unsqueeze(-1)
            state = self.input_to_coord(input)[..., 1:]
            dvds_term2 = diff_operators.jacobian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            dvds = dvds_term1 + dvds_term2
        elif self.deepReach_model == 'exact_sin':

            dvdt = (self.value_var / self.value_normto) * \
                (torch.sin(input[..., 0]/2)*dodi[..., 0] +
                 0.5*torch.cos(input[..., 0]/2)*output)

            dvds_term1 = (self.value_var / self.value_normto /
                          self.state_var.to(device=dodi.device)) * dodi[..., 1:] * torch.sin(input[..., 0]/2).unsqueeze(-1)
            state = self.input_to_coord(input)[..., 1:]
            dvds_term2 = diff_operators.jacobian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            dvds = dvds_term1 + dvds_term2
        elif self.deepReach_model == 'exact_exp':

            dvdt = (self.value_var / self.value_normto) * \
                ((-torch.exp(-5*input[..., 0])+1)*dodi[...,
                 0] + 5*torch.exp(-5*input[..., 0])*output)

            dvds_term1 = (self.value_var / self.value_normto /
                          self.state_var.to(device=dodi.device)) * dodi[..., 1:] * (-torch.exp(-5*input[..., 0])+1).unsqueeze(-1)
            state = self.input_to_coord(input)[..., 1:]
            dvds_term2 = diff_operators.jacobian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            dvds = dvds_term1 + dvds_term2

        elif self.deepReach_model == 'reg':
            dvdt = (self.value_var / self.value_normto) * dodi[..., 0]
            dvds = (self.value_var / self.value_normto /
                    self.state_var.to(device=dodi.device)) * dodi[..., 1:]
        else:
            raise NotImplementedError
        return torch.cat((dvdt.unsqueeze(dim=-1), dvds), dim=-1)

    # convert model io to real dv
    # TODO: need implementation for exact BC model and exact diff BC model
    def io_to_2nd_derivative(self, input, output):
        hes = diff_operators.batchHessian(
            output.unsqueeze(dim=-1), input)[0].squeeze(dim=-2)

        if self.deepReach_model == 'diff':
            vis_term1 = (self.value_var / self.value_normto /
                         self.state_var.to(device=hes.device))**2 * hes[..., 1:]
            state = self.input_to_coord(input)[..., 1:]
            vis_term2 = diff_operators.batchHessian(self.boundary_fn(
                state).unsqueeze(dim=-1), state)[0].squeeze(dim=-2)
            hes = vis_term1 + vis_term2

        else:
            hes = (self.value_var / self.value_normto /
                   self.state_var.to(device=hes.device))**2 * hes[..., 1:]

        return hes

    def set_model(self, deepreach_model):
        self.deepReach_model = deepreach_model
    # ALL FOLLOWING METHODS USE REAL UNITS

    @abstractmethod
    def state_test_range(self):
        raise NotImplementedError

    @abstractmethod
    def control_range(self, state):
        raise NotImplementedError

    @abstractmethod
    def equivalent_wrapped_state(self, state):
        raise NotImplementedError

    @abstractmethod
    def dsdt(self, state, control, disturbance):
        raise NotImplementedError

    @abstractmethod
    def boundary_fn(self, state):
        raise NotImplementedError

    @abstractmethod
    def sample_target_state(self, num_samples):
        raise NotImplementedError

    @abstractmethod
    def cost_fn(self, state_traj):
        raise NotImplementedError

    @abstractmethod
    def hamiltonian(self, state, dvds):
        raise NotImplementedError

    @abstractmethod
    def optimal_control(self, state, dvds):
        raise NotImplementedError

    @abstractmethod
    def optimal_disturbance(self, state, dvds):
        raise NotImplementedError

    @abstractmethod
    def plot_config(self):
        raise NotImplementedError

    def dsdt_(self, state, control, disturbance, ts):
        # torch.set_printoptions(precision=2, threshold=5000)

        dsdt = self.dsdt(state, control, disturbance)
        time_up = (ts > 0)*1.0

        state_test_range_ = torch.tensor(self.state_test_range()).cuda()
        # print(state[:100, :])
        # print(state.shape)
        output1 = torch.any(state < state_test_range_[
                            :, 0]-0.01, -1, keepdim=False)
        output2 = torch.any(state > state_test_range_[
                            :, 1]+0.01, -1, keepdim=False)
        out_of_range_index = torch.logical_or(output1, output2)

        dsdt[out_of_range_index] = 0.0
        return dsdt*time_up


class Drone6DConditioned(Dynamics):
    def __init__(self):
        self.g=9.8
        self.collision_R = 0.165
        self.arm_l=0.04
        self.roll_max=0.4
        self.pitch_max=0.4
        self.f_g_diff_max=1.5 # 5.8 <= u_T <= 13.8

        
        # self.obstacles= [[-0.245,1.5],[0.085,1.5],[-0.585,2.85],[-0.255,2.85]]
        self.obstacles= [[1.5,0.085],[2.85,-0.585],[1.5,-0.245],[2.85,-0.255]]
        # self.obstacles= [[1.5,0.085],[2.85,-0.585]]
        super().__init__(
            name='Drone6DConditioned', loss_type='brt_hjivi', set_mode="avoid",
            state_dim=7, input_dim=8, control_dim=3, disturbance_dim=3,
            state_mean=[2,-0.25, 0.5, 0, 0, 0, 0.25],
            state_var=[2,1, 0.5,1.5,1.5,0.3, 0.25],
            value_mean=0.9,
            value_var=1.2,
            value_normto=0.02,
            deepReach_model='exact',
            exact_factor=1,
        )

    def control_range(self, state):
        raise NotImplementedError

    def state_test_range(self):
        return [
            [-0, 4],
            [-1.25, 0.75],
            [0, 1],
            [-1.5, 1.5],
            [-1.5, 1.5],
            [-0.3, 0.3],
            [0, 0.5]
        ]

    def equivalent_wrapped_state(self, state):
        return torch.clone(state)

    # Dubins3D dynamics
    # \dot x    = v \cos \theta
    # \dot y    = v \sin \theta
    # \dot \theta = u
    def dsdt(self, state, control, disturbance):
        dsdt = torch.zeros_like(state)
        dsdt[..., 0] = state[...,3]
        dsdt[..., 1] = state[...,4]
        dsdt[..., 2] = state[...,5]
        dsdt[..., 3] = self.g*torch.tan(control[...,2]-disturbance[...,2])
        dsdt[..., 4] = -self.g*torch.tan(control[...,1]-disturbance[...,1])
        dsdt[..., 5] = (control[...,0]-disturbance[...,0])-self.g

        return dsdt

    def boundary_fn(self, state):
        dist=torch.ones_like(state[...,-1], device=state.device)*9999
        for i in range(len(self.obstacles)):
            
            p=state[..., :2]*1.0
            p[...,0]-=self.obstacles[i][0]
            p[...,1]-=self.obstacles[i][1]
            dist = torch.minimum(torch.norm(p, dim=-1)-self.arm_l,dist)
            

        return torch.maximum(dist, torch.zeros_like(dist)) - self.collision_R

    def sample_target_state(self, num_samples):
        raise NotImplementedError

    def cost_fn(self, state_traj):
        return torch.min(self.boundary_fn(state_traj), dim=-1).values

    def hamiltonian(self, state, dvds):
        control_percentage=torch.ones_like(state[...,6])-state[...,6]
        ham =  dvds[..., 0]*state[..., 3] + dvds[..., 1]*state[..., 4] + dvds[..., 2]*state[..., 5]
        ham += self.g*torch.abs(dvds[..., 3]*torch.tan(self.pitch_max*control_percentage)) + \
            self.g*torch.abs(dvds[..., 4]*torch.tan(self.roll_max*control_percentage)) + \
            torch.abs(dvds[..., 5]*(self.f_g_diff_max*control_percentage))
        return ham

    def optimal_control(self, state, dvds):
        u1= torch.sign(dvds[..., 5])*self.f_g_diff_max+self.g
        u2= -torch.sign(dvds[..., 4])*self.roll_max
        u3= torch.sign(dvds[..., 3])*self.pitch_max
        return torch.cat((u1[..., None], u2[..., None], u3[..., None]), dim=-1)

    def optimal_disturbance(self, state, dvds):
        d1= -torch.sign(dvds[..., 5])*self.f_g_diff_max*state[...,6]
        d2= torch.sign(dvds[..., 4])*self.roll_max*state[...,6]
        d3= -torch.sign(dvds[..., 3])*self.pitch_max*state[...,6]
        return torch.cat((d1[..., None], d2[..., None], d3[..., None]), dim=-1)

    def plot_config(self):
        return {
            'state_slices': [0, 0, 0, 1.5, 0, 0, 0],
            'state_labels': ['x', 'y', 'z', 'vx', 'vy', 'vz', 'dstb'],
            'x_axis_idx': 0,
            'y_axis_idx': 1,
            'z_axis_idx': 6,
        }
    
