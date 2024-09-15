import torch
import dynamics
import modules
import math
import matplotlib.pyplot as plt
import numpy as np

tMax = 1.4

dstb_percentage = 10.0/25.0 # range: [0,0.5]
gravity_constant = 9.8

current_time = 0
sim_time = 10
dt = 0.001
# [x, y, z, vx, vy, vz]
state = torch.tensor([2.5335283279418945,-0.25068557262420654,0.41,1.0,0.0,0.0]) # -0.25068557262420654 2.5335283279418945
# state = torch.tensor([0.0,0.0,0.41,0.0,0.0,0.0])
# [vz, roll, pitch]
control = torch.zeros(3)

value_threshold = 0.0 # Determines when safety filter should activate

nominal_pitch_control = math.radians(5.0) #Degrees
max_roll_bound = math.radians(15.0) # Degrees
max_pitch_bound = math.radians(15.0) # Degrees

w11 = [(0.245, 1.67), (-0.585, 3.02)]
w22 = [(-0.085, 1.67), (0.245, 1.67), (-0.395, 3.02), (-0.715, 3.02)]
w23 = [(-0.085, 1.67), (0.245, 1.67), (-0.385, 3.02), (-0.705, 3.02), (-0.065, 3.02)]
closer = [(-0.085, 1.67), (0.245, 1.67), (-0.255, 2.62), (-0.585, 2.62)]
radius = 0.165
obst_color = 'silver'

def draw_im_obstacles(fig):
    ax = fig.gca()
    for circle in w22:
        patch = plt.Circle(circle, radius, color=obst_color)
        ax.add_patch(patch)


def plot(states):
    fig = plt.figure(figsize=(2,4))
    draw_im_obstacles(fig)
    
    for point in states:
        # print(point[1], point[0])
        plt.scatter(point[1], point[0], s=2.0, c='grey')
        
    # plt.colorbar(orientation='vertical',shrink=0.4)
    
    ax = plt.gca()
    ax.grid(True)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim([-1, 1])
    ax.set_ylim([0, 5])
    ax.set_aspect('equal')

    plt.title("Safety Filter")

    plt.tight_layout()
    # plt.savefig(directory + "_traj.jpg", dpi=800, bbox_inches='tight') 
    
    plt.show()

def step_dynamics(state, roll, pitch, dt):
    """
    Step the dynamics forward base on a single euler step:
        State: [x, y, z, vx, vy, vz]
    """
    
    state[0] = state[0] + state[3] * dt;
    state[1] = state[1] + state[4] * dt;
    # next_state[2] = next_state[2] + next_state[5] * dt; We do not care about Z in this case.
    state[3] = state[3] + gravity_constant * math.tan(pitch);
    state[4] = state[4] - gravity_constant * math.tan(roll);

    # invert y and vy because of the coordinate system of reach
    state[1] = state[1];
    state[4] = state[4];
    
    
def get_opt_ctrl(state, tMax, dstb_percentage, model,dynamics_):

  coords_tensor=torch.zeros(8)
  coords_tensor[0]=tMax
  coords_tensor[1:7]=state
  coords_tensor[7]=dstb_percentage

  coords_tensor[1:] = torch.clamp(coords_tensor[1:], torch.FloatTensor(dynamics_.state_test_range(
        ))[..., 0], torch.FloatTensor(dynamics_.state_test_range())[..., 1])
  coords_tensor.requires_grad_(True)
  results = model({'coords': dynamics_.coord_to_input(coords_tensor).unsqueeze(0)})
  dvs = dynamics_.io_to_dv(
        results["model_in"], results["model_out"].squeeze(dim=-1)).detach()
  
  control = dynamics_.optimal_control(
        coords_tensor[1:], dvs[..., 1:])
  
  return control.detach()

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

def get_value(state, tMax, dstb_percentage, model,dynamics_):
    coords_tensor=torch.zeros(8)
    coords_tensor[0]=tMax
    coords_tensor[1:7]=state
    coords_tensor[7]=dstb_percentage
    
    coords_tensor[1:] = torch.clamp(coords_tensor[1:], torch.FloatTensor(dynamics_.state_test_range(
        ))[..., 0], torch.FloatTensor(dynamics_.state_test_range())[..., 1])
    coords_tensor.requires_grad_(True)
    results = model({'coords': dynamics_.coord_to_input(coords_tensor).unsqueeze(0)})
    value = dynamics_.io_to_value(
            results["model_in"], results["model_out"].squeeze(dim=-1)).detach()
    
    return value


dynamics_=dynamics.Drone6DConditioned()
model = modules.SingleBVPNetEval(in_features=dynamics_.input_dim, out_features=1, type='sine', mode='mlp',
                                     final_layer_factor=1., hidden_features=64, num_hidden_layers=3)
model.load_state_dict(torch.load(
        "conversion/reach_model_w_walls/model_harder_sine.pth", map_location=('cpu') )["model"])
model.eval()

# state=torch.randn(6)
state_plot = []

dynamics_.roll_max = max_roll_bound
dynamics_.pitch_max = max_pitch_bound

while(current_time < sim_time):
    #Used for plotting
    current_state = [state.numpy()[0], state.numpy()[1], state.numpy()[2]]
    state_plot.append(current_state)
    
    # print(state,get_opt_dstb(state,tMax,dstb_percentage,model,dynamics_))
    # print(state,get_opt_ctrl(state,tMax,dstb_percentage,model,dynamics_))
    current_value = get_value(state,tMax,dstb_percentage,model,dynamics_)
    
    if (current_value < value_threshold) and (state[0] > 0.2):
        # Find the optimal control
        control = get_opt_ctrl(state,tMax,dstb_percentage,model,dynamics_)[0]
        # print("Safety Filter Control: ", control)
    else:
        # [vz, roll, pitch]
        control[2] = 1.0 * nominal_pitch_control
    # print(control)
    step_dynamics(state=state, 
                    roll=control[1],
                    pitch=control[2],
                    dt=dt)
    
    # print(state)
    current_time = current_time + dt
    control = torch.zeros(3)

    if (state[0] > 3.5):
        break
    
np.save("sim_filter_states", np.array(state_plot))

plot(states=state_plot)