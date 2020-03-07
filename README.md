# UnityMachineLearningForProjectButterfly

Aviv Elor - aelor@ucsc.edu - avivelor1@gmail.com

An exploration of Unity ML Agents on training a "robo arm" to protect butterflies with bubble shields in an immersive virtual environment.
This project utilizes Reinforcement Learning through Proximal Policy Optimization to train a Virtual Robot Arm to play an Immersive Virtual Reality Physical Exercise Game.
Overall, this was a fun deep dive into exploring Machine Learning through mlagents -- the project could definitely be polished if I have more time.
Feel free to try out the standalone build if you want attempt to predict torque values for two joints better than a neural network!

If any questions, email aelor@ucsc.edu and or message Aviv Elor.

To mess around with training, refer to the Unity ML-Agents Documentation at https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Readme.md.
In a nut shell, download Anaconda and setup a virtual environment through conda activate mlagents (see this example to configure your environment https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Readme.md).
Enter the training scene at the following:

```sh
~\UnitySDK\Assets\ML-Agents\Examples\CM202-ExoArm\Scenes\
```

With the anaconda terminal, prepare to train through using the following terminal command:

```sh
mlagents-learn config/trainer_config.yaml --run-id=<run-identifier> --train --time-scale=100
```

Now sit back and let the model train. After checkpoints are saved, you can use tensorboard to examine the model's performance:

```sh
tensorboard --logdir=summaries
```

Subsquently, I was able to train the robot arm very well through utilizing 16 agents in parallel through Deep Reinforcement Learning.
After four hours of training, my reward slowly rose from 0.1 to 40 (where +0.01 reward was given per every frame the arm successfuly protected the butterfly).
See the demo video below for a discussion of the training, results, and demo experience.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/5J7xes28bZA/0.jpg)](http://www.youtube.com/watch?v=5J7xes28bZA "Demo Video")

Demo Materials:
* Standalone Downloadable Demo (Stable) - https://github.com/avivelor/UnityMachineLearningForProjectButterfly/raw/master/UnitySDK/ExoButterfly-StandaloneBuild.zip
* Video Demo - https://youtu.be/5J7xes28bZA
* Blog Posts - https://www.avivelor.com/

External Tools Used and Modified for this Project:
* Unity Machine Learning Agents Beta - https://github.com/Unity-Technologies/ml-agents
* Project Butterfly - https://www.avivelor.com/post/project-butterfly
* Unity ML Agents Introduction - https://towardsdatascience.com/an-introduction-to-unity-ml-agents-6238452fcf4c
* Unity ML Agents Reacher Example - https://github.com/Unity-Technologies/ml-agents/tree/master/Project/Assets/ML-Agents/Examples/Reacher
* Older Unity ML Reacher Example by PHRABAL  - https://github.com/PHRABAL/DRL-Reacher
* Proximal Policy Optimization (PPO) - https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Training-PPO.md
* Deep Reinforcement Learning (through Deep Deterministic Policy Gradient or DDPG) -  https://arxiv.org/pdf/1509.02971.pdf

Reading References:
* Unity Machine Learning - https://unity3d.com/machine-learning
* Academic Paper on Project Butterfly at IEEEVR 2019 Paper by Elor et Al - https://www.researchgate.net/publication/335194991_Project_Butterfly_Synergizing_Immersive_Virtual_Reality_with_Actuated_Soft_Exosuit_for_Upper-Extremity_Rehabilitation
