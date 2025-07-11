import torch
import torch.nn as nn
from skrl.models.torch import DeterministicMixin
import os

class DeterministicActor(nn.Module, DeterministicMixin):
    def __init__(self, input_dims, n_actions, device):
        super().__init__()
        self.device = device

        self.net_container = nn.Sequential(
            nn.Linear(input_dims, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU()
        ).to(self.device)

        self.policy_layer = nn.Linear(32, n_actions).to(self.device)
        self.log_std_parameter = nn.Parameter(torch.zeros(n_actions))  # if checkpoint has it

        DeterministicMixin.__init__(self)

    def forward(self, observation):
        x = self.net_container(observation)
        mean_actions = self.policy_layer(x)
        return {"mean_actions": mean_actions}


class ActorNetwork:
    def __init__(self, input_dims, n_actions, alpha=3e-4, chkpt_dir=""):
        self.input_dims = input_dims
        self.n_actions = n_actions
        self.chkpt_dir = chkpt_dir
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Create the deterministic actor model
        self.policy = DeterministicActor(self.input_dims, self.n_actions, self.device).to(self.device)

    def load_checkpoint(self):
        ckpt_path = os.path.join(self.chkpt_dir)
        checkpoint = torch.load(ckpt_path, map_location=self.device)

        policy_state_dict = checkpoint.get("policy", None)
        if policy_state_dict is None:
            raise KeyError("Checkpoint does not contain 'policy' weights")

    # Filter out value_layer keys
        filtered_policy_state_dict = {k: v for k, v in policy_state_dict.items() if not k.startswith("value_layer")}

        self.policy.load_state_dict(filtered_policy_state_dict)
        print(f"[INFO] Loaded policy from {ckpt_path}")

    def eval(self):
        self.policy.eval()

    def __call__(self, obs_tensor):
        with torch.no_grad():
            action = self.policy(obs_tensor)["mean_actions"]
        return action
