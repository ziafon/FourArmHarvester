import gym
import numpy as np
from stable_baselines3 import DQN

def main():
    env = gym.make('CartPole-v1')
    n_actions = env.action_space.n
    n_states = env.observation_space.shape[0]
    model = DQN('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=50000)
    state = env.reset()
    done = False
    while not done:
        action, _states = model.predict(state)
        state, reward, done, info = env.step(np.argmax(action))
        if done:
            print('Episode finished after {} timesteps'.format(info['episode']))
            break
    model.save('dqn_cartpole')
    env.close()


