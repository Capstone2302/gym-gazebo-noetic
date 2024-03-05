#!/usr/bin/env python3
import gym
from gym import wrappers
import gym_gazebo
import time
#import numpy
import random
import time

import liveplot

import os.path
from os import path

import inspect

from collections import namedtuple
import numpy as np
from tensorboardX import SummaryWriter

import torch
import torch.nn as nn
import torch.optim as optim
from datetime import datetime

# Logging dependencies
import cv2
import os
import signal
import sys
import shutil

HIDDEN_SIZE = 128 # number of neurons in hidden layer
BATCH_SIZE = 16   # number of episodes to play for every network iteration
PERCENTILE = 70   # only the episodes with the top 30% total reward are used 
                  # for training

class Net(nn.Module):
    '''
    @brief Takes an observation from the environment and outputs a probability 
           for each action we can take.
    '''
    def __init__(self, obs_size, hidden_size, n_actions):
        super(Net, self).__init__()

        # Define the NN architecture
        #
        # REMINDER:
        # as the last layer outputs raw numerical values instead of 
        # probabilities, when we later in the code use the network to predict
        # the probabilities of each action  we need to pass the raw NN results 
        # through a SOFTMAX to get the probabilities.
        self.net = nn.Sequential(
            nn.Linear(obs_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, n_actions)
        )

        self.folderName = datetime.now().strftime("%b%d-%H-%M-%S-rlwheel")

    def forward(self, x):
        return self.net(x)


# Stores the total reward for the episode and the steps taken in the episode
Episode = namedtuple('Episode', field_names=['reward', 'steps'])
# Stores the observation and the action the agent took
EpisodeStep = namedtuple('EpisodeStep', field_names=['observation', 'action', 'PID_action'])


def iterate_batches(env, net, batch_size):
    '''
    @brief a generator function that generates batches of episodes that are
           used to train NN on
    @param env: environment handler - allows us to reset and step the simulation
    @param net: neural network we use to predict the next action
    @param batch_size: number of episodes to compile
    @retval batch: returns a batch of batch_size episodes (each episode contains
                  a list of observations and actions and the total reward for
                  the episode)
    '''
    batch = [] # a list of episodes
    episode_reward = 0.0 # current episode total reward
    episode_steps = [] # list of current episode steps
    sm = nn.Softmax(dim=1) # SOFTMAX object - we use it to convert raw NN 
                           # outputs to probabilities

    # Reset environment and obtain the first observation. An observation 
    # consists of a 4 element tuple with position and angle information:
    #   * x
    #   * x_dot
    #   * theta
    #   * theta_dot
    obs = env.reset()

    # Every iteration we send the current observation to the NN and obtain
    # a list of probabilities for each action
    action_num = 0
    while True:
        action_num += 1
        # Convert the observation to a tensor that we can pass into the NN
        # print("action num: " + str(action_num) + " obs: " + str(obs))
        obs_v = torch.FloatTensor([obs])

        # Run the NN and convert its output to probabilities by mapping the 
        # output through the SOFTMAX object.
        act_probs_v = (net(obs_v))

        # Unpack the output of the NN to extract the probabilities associated
        # with each action.
        # 1) Extract the data field from the NN output
        # 2) Convert the tensors from the data field into numpy array
        # 3) Extract the first element of the network output. This is where 
        #    the probability distribution are stored. The second element of the
        #    network output stores the gradient functions (which we don't use) 
        # act_probs = act_probs_v.data.numpy()[0]
        # print('act_probs ',act_probs)
        
        # Sample the probability distribution the NN predicted to choose
        # which action to take next.
        action = float(act_probs_v[0])

        # Run one simulation step using the action we sampled.
        next_obs, reward, is_done, _ = env.step(action)

        # Process the simulation step:
        #   - add the current step reward to the total episode reward
        #   - append the current episode
        episode_reward += reward

        # Add the **INITIAL** observation and action we took to our list of  
        # steps for the current episode
        # print('action: ', action)
        PID_action=PID_control(obs)
        print("obs: ", obs)
        print('PID: ', PID_action)
        print('action: ', action, '\n')
        episode_steps.append(EpisodeStep(observation=obs, action=action, PID_action=[PID_action]))

        # When we are done with this episode we will save the list of steps in 
        # the episode along with the total reward to the batch of episodes 
        # our NN will train on next time (actually the NN will train only on 
        # the top X% of the highest rewarded episodes).
        #
        # We then reset our variables and environment in preparation for the 
        # next episode.
        if is_done:
            batch.append(Episode(reward=episode_reward, steps=episode_steps))
            episode_reward = 0.0
            episode_steps = []
            next_obs = env.reset()
            action_num = 0

            # If we accumulated enough episodes in the batch of episodes we 
            # pass the batch of episodes to the caller of this function. This
            # will allow the NN to train on the top X% of the highest rewarded
            # episodes.
            if len(batch) == batch_size:
                yield batch
                batch = []

        # if we are not done the old observation becomes the new observation
        # and we repeat the process
        obs = next_obs
def PID_control(obs):
    error = obs[0]
    prev_err = obs[1]
    dt = obs[2]
    derivative = (error - prev_err) / dt

    # Calculate control output with PID terms
    proportional = (2.3) * error
    derivative = (1) * derivative

    return -(proportional + derivative)


def filter_batch(batch, percentile):
    '''
    @brief given a batch of episodes it determines which are the "elite" 
           episodes in the top percentile of the batch based on the episode
           reward
    @param batch:
    @param percentile:
    @retval train_obs_v: observation associated with elite episodes
    @retval train_act_v: actions associated with elite episodes (mapped to 
                         observations above)
    @retval reward_bound: the threshold reward over which an episode is 
                          considered elite - used for monitoring progress
    @retval reward_mean: mean reward - used for monitoring progress
    '''

    # Extract each episode reward from the batch of episodes
    rewards = list(map(lambda s: s.reward, batch))

    # Determine what is the threshold reward (the reward_bound) above which
    # an episode is considered "elite" and will be used for training
    reward_bound = np.percentile(rewards, percentile)
    
    # Calculate the mean of the reward for all the episodes in the batch. We
    # use this as an indicator for how well the training is progressing. We 
    # hope the mean reward will trand higher as training progresses.
    reward_mean = float(np.mean(rewards))
    
    # We will accumulate the observations and actions we want to train on in 
    # the train_obs and train_act variables
    train_obs = []
    train_act = []

    # For each episode in the batch determine if the episode is an "elite" 
    # episode (it has a reward above the threshold reward_bound). If this is 
    # the case add the episodes observations and action to the train_obs and 
    # train_act
    for example in batch:
        # if example.reward < reward_bound:
        #     continue
        # We reach here if the episode is "elite"
        # adds the observations and actions from each episode to our training
        # sets (map iterates over each step in examples.steps and passes it 
        # to the lambda function which returns either the observation or the 
        # action of the step)
        # print(example.steps)
        train_obs.extend(map(lambda step: step.observation, example.steps))
        train_act.extend(map(lambda step: step.PID_action, example.steps))

    # Convert the observations and actions into tensors and return them to be  
    # used to train the NN
    train_obs_v = torch.FloatTensor(train_obs)
    train_act_v = torch.FloatTensor(train_act)
    return train_obs_v, train_act_v, reward_bound, reward_mean

# Function to handle interrupt signal
def handle_interrupt(signum, frame, folderName, net, record):
    print("Interrupt signal received. Creating video...")
    output_dir = 'runs/video/images'
    output_video_path = 'runs/video/'+folderName+'.mp4'
    fps = 30 # TODO: Might not be accurate
    if record:
        videoWriter(output_dir, output_video_path, fps)
    torch.save(net.state_dict(), 'runs/model/'+folderName+'.pth')
    print("Cleanup complete. Exiting.")
    os._exit(0)

def videoWriter(output_dir, output_video_path, fps):
    if os.path.exists(output_dir):
        create_video(output_dir, output_video_path, fps)

        delete_output_directory(output_dir)

def create_video(output_dir, output_video_path, fps):
    images = [img for img in os.listdir(output_dir) if img.endswith(".png")]
    images.sort()

    frame = cv2.imread(os.path.join(output_dir, images[0]))
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    for image in images:
        img_path = os.path.join(output_dir, image)
        img = cv2.imread(img_path)
        video.write(img)

    cv2.destroyAllWindows()
    video.release()

# Function to delete the entire output directory
def delete_output_directory(output_dir):
    try:
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
    except Exception as e:
        print(f"Error deleting directory {output_dir}: {e}")

if __name__ == '__main__':
    # Setup environment
    env = gym.make('GazeboWheel-v1')
    record = True
    env.setRecordingState(record)
    obs_size = env.observation_space.shape[0]
    n_actions = env.action_space.n

    folderName = datetime.now().strftime("%b%d-%H-%M-%S-rlwheel")
    # outdir = 'runs/video/'+folderName
    # env = gym.wrappers.Monitor(env, directory=outdir, force=True)
    # plotter = liveplot.LivePlot(outdir)

    # Create the NN object
    net = Net(obs_size, HIDDEN_SIZE, n_actions)
    net.load_state_dict(torch.load('runs/model/Mar04-18-50-22-rlwheel.pth'))

    signal.signal(signal.SIGINT, lambda signum, frame: handle_interrupt(signum, frame, folderName, net, record))
    # PyTorch module that combines softmax and cross-entropy loss in one 
    # expresion
    objective = nn.MSELoss()
    optimizer = optim.Adam(params=net.parameters(), lr=0.01)
    # Tensorboard writer for plotting training performance
    writer = SummaryWriter(logdir='runs/tensorboard/'+folderName,comment="-wheel")
    delete_output_directory('runs/video/images')
    os.makedirs('runs/video/images', exist_ok=True)

    # For every batch of episodes (BATCH_SIZE episodes per batch) we identify the
    # episodes in the top (100 - PERCENTILE) and we train our NN on them.
    for iter_no, batch in enumerate(iterate_batches(env, net, BATCH_SIZE)):
        print("**** TRAINING ****")
        # Identify the episodes that are in the top PERCENTILE of the batch
        obs_v, acts_v, reward_b, reward_m = filter_batch(batch, PERCENTILE)
        #replace acts_v with output from PID control 

        # **** TRAINING OF THE NN ****
        # Prepare for training the NN by zeroing the acumulated gradients.
        optimizer.zero_grad()

        # Calculate the predicted probabilities for each action in the best 
        # episodes
        action_scores_v = net(obs_v)

        # Calculate the cross entropy loss between the predicted actions and 
        # the actual actions
        
        loss_v = objective(action_scores_v, acts_v)

        # Train the NN: calculate the gradients using loss_v.backward() and 
        # then adjust the weights based on the gradients using optimizer.step()
        loss_v.backward()
        optimizer.step()

        print("**** DONE TRAINING *****")

        # **** END OF TRAINING ****

        # Display summary of current batch
        print("%d: loss=%.3f, reward_mean=%.1f, reward_bound=%.1f" % (
            iter_no, loss_v.item(), reward_m, reward_b))
        # Save tensorboard data
        writer.add_scalar("loss", loss_v.item(), iter_no)
        writer.add_scalar("reward_bound", reward_b, iter_no)
        writer.add_scalar("reward_mean", reward_m, iter_no)

        # When the reward is sufficiently large we consider the problem has
        # been solved
        if reward_m > 1000:
            print("Solved!")

            output_dir = 'runs/video/images'
            output_video_path = 'runs/video/'+folderName+'.mp4'
            fps = 30 # TODO: Might not be accurate
            if record:
                videoWriter(output_dir, output_video_path, fps)
            torch.save(net.state_dict(), 'runs/model/'+folderName+'.pth')
            break
    writer.close()