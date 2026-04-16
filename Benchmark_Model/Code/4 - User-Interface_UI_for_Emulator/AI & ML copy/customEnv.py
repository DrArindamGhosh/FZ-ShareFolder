import numpy as np
import tensorflow as tf
from keras import layers
from keras.optimizers import Adam
import win32com.client

# Define the Custom Simulation Environment
class CustomSimulationEnv:
    def __init__(self, tracks_df):
        self.tracks_df = tracks_df
    
    def reset(self):
        # Reset simulation to start conditions, initializing vehicle states
        self.Vx = 0.0  # Initialize velocity in x
        self.Vy = 0.0  # Initialize velocity in y
        self.yawAngle = 0.0  # Initialize yaw angle
        self.x = 0.0  # Initialize x position
        self.y = 0.0  # Initialize y position
        
        # Return the initial state
        return np.array([self.Vx, self.Vy, self.yawAngle, self.x, self.y])
    
    def step(self, action):
        # Submit action to simulation (steering input), and receive the new vehicle state
        new_state = self.submit_action_to_simulation(action)
        balance.DynamicsSimExtDrive2(carHandle, steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, 0, 0)

        # Position
        x, y, z = balance.ReadChannelVector(carHandle, "Chassis", "", "Position")
        # Yaw Rates
        yawAngle = balance.ReadChannelScalar(carHandle, "Chassis", "", "Vehicle Yaw")
        # Velocities
        Vx, Vy, Vz = balance.ReadChannelVector(carHandle, "Chassis", "", "Vehicle Velocity")

        e_phi, total_saturated_e_l = self.calculate_steering_control(self.tracks_df, self.Vx, self.Vy, self.yawAngle, self.x, self.y)
        
        # Compute reward based on the new state (lateral and angular offsets)
        reward = self.compute_reward(e_phi, total_saturated_e_l)
        
        # Check if the episode is done (e.g., crash or completion of track)
        done = self.check_if_done(new_state)
        
        return new_state, reward, done

    def calculate_steering_control(tracks_df, Vx, Vy, yawAngle, x, y):
        
        timestep = 0.002
        e_phi_gain = 2
        nLap = 0  
        newSim = True 
        e_l_gains = [3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04]
        e_l_limits = [1.5, 1.5, 2, 2.75, 3, 2.75, 2.25, 1.7]  
        total_e_l_limits = 10
        
        # Initialize the trajectory matrix from the track data
        trajectory_matrix = tracks_df

        # Reference State Now
        ref_distance, refDistances, newSim = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, newSim, nLap)

        # Calculate Lever Distances
        lever_distances = LeverDistances(Vx)

        # Corresponding Ideal Path Points
        ideal_path_points, ideal_coordinates_plot = CorrespondingIdealPathPoints(trajectory_matrix, lever_distances, ref_distance, nLap)

        # Lever Absolute Coordinates
        lever_absolute_coordinates, lever_coords_plot = LeverAbsoluteCoordinates(lever_distances, x, y, yawAngle)

        # Angular Offset (e_phi)
        e_phi, refLine_orientation = AngularOffset(trajectory_matrix, ref_distance, yawAngle, e_phi_gain, nLap)

        # Lateral Offset (e_l)
        e_l_gains = np.array(e_l_gains)
        e_l_upper_limits = np.array(e_l_limits)
        e_l_lower_limits = np.array([-limit for limit in e_l_limits])
        upper_total_e_l_limits = total_e_l_limits
        lower_total_e_l_limits = - total_e_l_limits
        
        e_l, saturated_e_l, total_saturated_e_l, error_values_gains_array = LateralOffset(
            ideal_path_points, lever_absolute_coordinates, yawAngle, 
            e_l_gains, e_l_lower_limits, e_l_upper_limits, lower_total_e_l_limits, upper_total_e_l_limits
        )

        return e_phi, total_saturated_e_l  # Return the angular and lateral offsets

    
    def submit_action_to_simulation(self, action):
        # Submit the steering action to the simulation and return the new vehicle state
        # Placeholder for simulation interaction
        return np.random.rand(4)  # New state (position, velocity, etc.)
    
    def compute_reward(self, e_phi, total_saturated_e_l):
        # Compute the reward based on how close the vehicle is to the reference line
        
        reward = - (total_saturated_e_l ** 2 + e_phi ** 2)
        return reward
    
    def check_if_done(self, state):
        # Logic to determine if the episode is done (e.g., crash or finish)
        done = np.random.choice([True, False], p=[0.1, 0.9])  # Placeholder
        return done


# Define the PPO Agent
class CarRacingPPOAgent:
    def __init__(self, action_space=(-28, 28), learning_rate=0.0003, gamma=0.99):
        self.action_space = action_space
        self.gamma = gamma
        self.learning_rate = learning_rate
        self.actor, self.critic = self.build_model()

    def build_model(self):
        # Build Actor model
        state_input = layers.Input(shape=(4,))  # Assuming 4 state inputs (position, velocity, yaw, etc.)
        dense1 = layers.Dense(64, activation="relu")(state_input)
        dense2 = layers.Dense(64, activation="relu")(dense1)
        
        # Output layer for continuous action (steering)
        steering_output = layers.Dense(1, activation="tanh")(dense2)
        steering_output = layers.Lambda(lambda x: x * self.action_space[1])(steering_output)  # Scale to (-28, 28)
        
        actor = tf.keras.models.Model(inputs=state_input, outputs=steering_output)
        actor.compile(optimizer=Adam(learning_rate=self.learning_rate))

        # Build Critic model (State value prediction)
        critic_output = layers.Dense(1)(dense2)
        critic = tf.keras.models.Model(inputs=state_input, outputs=critic_output)
        critic.compile(optimizer=Adam(learning_rate=self.learning_rate))
        
        return actor, critic

    def select_action(self, state):
        # Select an action (steering angle) based on the current policy
        state = np.expand_dims(state, axis=0)  # Add batch dimension
        steering_angle = self.actor.predict(state)[0]
        return steering_angle

    def train(self, states, actions, rewards, next_states, dones):
        # Compute advantage and update actor and critic
        values = self.critic.predict(states)
        next_values = self.critic.predict(next_states)
        
        advantages = rewards + self.gamma * next_values * (1 - dones) - values
        advantages = np.squeeze(advantages)
        
        # Update Critic
        self.critic.train_on_batch(states, rewards)
        
        # Update Actor using the advantage
        with tf.GradientTape() as tape:
            action_probs = self.actor(states)
            actions = tf.convert_to_tensor(actions)
            actor_loss = -tf.reduce_mean(action_probs * advantages)  # Maximize the advantage
        
        actor_grads = tape.gradient(actor_loss, self.actor.trainable_weights)
        self.actor.optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_weights))

    def load(self, name):
        self.actor.load_weights(name + "_actor.h5")
        self.critic.load_weights(name + "_critic.h5")

    def save(self, name):
        self.actor.save_weights(name + "_actor.h5")
        self.critic.save_weights(name + "_critic.h5")


# Training Loop
if __name__ == "__main__":
    env = CustomSimulationEnv()
    agent = CarRacingPPOAgent()

    num_episodes = 1000
    for episode in range(num_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        
        while not done:
            # Select action from the agent
            action = agent.select_action(state)
            
            # Step in the simulation environment
            next_state, reward, done = env.step(action)
            
            # Train the agent with the experience
            agent.train(np.array([state]), np.array([action]), np.array([reward]), np.array([next_state]), np.array([done]))
            
            # Update state
            state = next_state
            episode_reward += reward
        
        print(f"Episode {episode} Reward: {episode_reward}")
        
        # Optionally save the agent model periodically
        if episode % 100 == 0:
            agent.save(f"ppo_agent_{episode}")
