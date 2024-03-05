# Feb02-15-13-01-rlwheel
- run on gazebo data
- started from Feb01-12-53-36-rlwheel.pth
- 35ms delay added

# Feb02-15-25-28-rlwheel 
- run on gazebo data
- started from Feb01-12-53-36-rlwheel.pth
- 35ms delay added
- RTF = 1
- reward decrease after 5 runs

# Feb02-15-50-29-rlwheel
- run on gazebo data
- started from Feb01-12-53-36-rlwheel.pth
- no delay
- RTF = 1
- BATCH_SIZE = 32 (increased from 16)
- PERCENTILE = 80 (increased from 70)

# Feb02-16-13-39-rlwheel
- run on gazebo data
- started from Feb01-12-53-36-rlwheel.pth
- no artificial delay
- RTF = 1
- BATCH_SIZE = 1000
- PERCENTILE = 80

# Feb02-19-54-02-rlwheel
- train on gazebo data
- started new model
- no artificial delay
- RTF = 1
- BATCH_SIZE = 16
- PERCENTILE = 80
- n_actions = 5
- add randomization to ball spawn
- removed csv writing for telemetry
- didn't train very well

# Feb02-20-32-31-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 80
- n_actions = 5


# Feb02-20-32-31-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 70
- n_actions = 301

# Feb02-23-18-49-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 1000
- PERCENTILE = 80
- n_actions = 301

# Feb03-10-40-54-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 80
- n_actions = 301

# Feb03-16-05-46-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 80
- n_actions = 301
- CHANGED REWARD SCHEME

# Feb03-17-50-00-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 80
- n_actions = 301
- reverted reward scheme

# Feb03-22-25-38-rlwheel
- train on gazebo data
- started new model
- RTF = 5
- BATCH_SIZE = 100
- PERCENTILE = 70
- n_actions = 301
- reverted reward scheme

# Feb04-11-35-46-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 70
- n_actions = 301

# Feb04-11-35-46-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 70
- n_actions = 301
- yikes, modest incline, followed by steep cliff

# Feb06-09-44-47-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 70
- n_actions = 5
- starts good and declines in reward mean, what the heck!!

# Feb06-10-40-27-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 16
- PERCENTILE = 70
- n_actions = 5
- NEW HAND TUNED REWARD SCHEME [-1, 1]


# Feb06-17-29-21-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 16
- PERCENTILE = 70
- HIDDEN_SIZE = 128
- layers = 2 (upped from 1)
- n_actions = 5
- rewards [-1, 1]
- FIXXED REALLY HIGH BALL VELOCITY
- changed how to get dt

# Feb06-18-19-23-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 16
- PERCENTILE = 70
- HIDDEN_SIZE = 128
- layers = 2
- n_actions = 5
- rewards [-1, 1]
- ball spawning only one side

# Feb06-18-45-33-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- BATCH_SIZE = 16
- PERCENTILE = 70
- HIDDEN_SIZE = 128
- layers = 1
- n_actions = 5
- rewards [-1, 1]
- ball spawning only one side

# Feb06-20-05-43-rlwheel
# Feb06-21-04-21-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- HIDDEN_SIZE = 128
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 5
- rewards [-1, 1]
- ball spawned left

# Feb06-21-42-43-rlwheel
- train on gazebo data
- started new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 5
- rewards [-1, 1]
- ball spawn one sided
- positions rounded

# Feb06-22-05-23-rlwheel
# 
- train on gazebo data
- started new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 5
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded

# Feb29-12-20-23-rlwheel
- PID Imitation learning!!
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: JointVelocityController

# Feb29-12-20--rlwheel
- PID Imitation learning!!
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: JointVelocityController


# Mar02-20-11-56-rlwheel
- PID Imitation learning!!
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: EffortJointController

# Mar03-15-12-54-rlwheel
# Mar03-15-42-28-rlwheel
# Mar03-16-13-26-rlwheel
- PID Imitation learning!!
- model started from Mar02-20-11-56-rlwheel
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- layers = 1
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: EffortJointController
- noticed plateauting

# Mar03-16-41-16-rlwheel
- PID Imitation learning!!
- new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- **layers = 2
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: EffortJointController

# Mar04-08-49-34-rlwheel
- PID Imitation learning!!
- new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- **layers = 3
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn random, uniform
- positions rounded
- Controller type: EffortJointController


# Mar04-11-44-03-rlwheel
- PID Imitation learning!!
- x_threshold = 0.08 (changed from 0.2)
- new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- PERCENTILE = 70
- **layers = 3
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn: centered
- positions rounded
- Controller type: EffortJointController

# Mar04-15-31-14-rlwheel
- PID Imitation learning!!
- x_threshold = 0.02
- new model
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- **layers = 4
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn: centered
- Controller type: EffortJointController

# Mar04-19-33-49-rlwheel
- decreased x reset to train only near equilibrium point (abs(x) < 0.02)
- scaled neural output and PID by 100x to increase MSE
- getting more garbage

# Mar04-20-32-32-rlwheel
- PID Imitation learning!!
- x_threshold = 0.02
- model starting from Mar04-18-50-22-rlwheel
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- **layers = 4
- n_actions = 1, continuous (MSE Loss)
- rewards [-1, 1]
- ball spawn: centered
- Controller type: EffortJointController

# Mar05-10-24-42-rlwheel
- bux fixed
- PID Imitation learning!!
- x_threshold = 0.2
- model starting from Mar04-18-50-22-rlwheel
- RTF = 1
- HIDDEN_SIZE = 256
- BATCH_SIZE = 16
- **layers = 4
- n_actions = 1, continuous (MSE Loss)
- rewards [0, 1], centered
- ball spawn: centered
- Controller type: EffortJointController

# To Try:
- try more action
- rerun PID
- add pause
- change action scaling
- do csv writing during pause or remove csv writing
- get rid of ball velocity
