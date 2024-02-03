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

# 
- train on gazebo data
- started new model
- no artificial delay
- RTF = 1
- BATCH_SIZE = 100
- PERCENTILE = 80
- n_actions = 5

To Try:
# try more action
# add pause
# change action scaling
# do csv writing during pause or remove csv writing
