
# BALSA:  BAyesian Learning-based Safety and Adaptation

Fan, David D., Jennifer Nguyen, Rohan Thakker, Nikhilesh Alatur, Ali-akbar Agha-mohammadi, and Evangelos A. Theodorou. "Bayesian Learning-Based Adaptive Control for Safety Critical Systems." [arXiv preprint arXiv:1910.02325](https://arxiv.org/abs/1910.02325) (2019).

### Paper Abstract
Deep learning has enjoyed much recent success, and applying state-of-the-art model learning methods to controls is an exciting prospect. However, there is a strong reluctance to use these methods on safety-critical systems, which have constraints on safety, stability, and real-time performance. We propose a framework which satisfies these constraints while allowing the use of deep neural networks for learning model uncertainties. Central to our method is the use of Bayesian model learning, which provides an avenue for maintaining appropriate degrees of caution in the face of the unknown. In the proposed approach, we develop an adaptive control framework leveraging the theory of stochastic CLFs (Control Lypunov Functions) and stochastic CBFs (Control Barrier Functions) along with tractable Bayesian model learning via Gaussian Processes or Bayesian neural networks. Under reasonable assumptions, we guarantee stability and safety while adapting to unknown dynamics with probability 1. We demonstrate this architecture for high-speed terrestrial mobility targeting potential applications in safety-critical high-speed Mars rover missions.

## Getting started
The `master` branch contains code for running the controller on hardware, as well as replicating the toy dynamics plots from the paper.  The `sim_plots` branch contains code for running the gazebo simulation.

### Prerequisites
The following packages can be installed with `pip`:
* `numpy`
* `scipy`
* `progress`
* `osqp`
* `GPy`
* `tensorflow`
* `keras`
* `matplotlib`

You will also need ROS `melodic` and `gazebo`.  Also `rqt_multiplot_plugin` is used for plotting.  You will also need `tmux` and `tmuxp` for running the start-up scripts.

### Plotting
To replicate the plots from the paper, run:

```
./src/test_adaptive_clbf.py
```

### Simulation
To run the simulation (and obtain plots), first checkout the `sim_plots` branch.  Then run:
```
tmuxp load scripts/startup_sim.yaml
```

### Hardware
For the on-board computer, run 

```
tmuxp load scripts/startup_hw.yaml
```

and for the base station computer, run

```
tmuxp load scripts/startup_base.yaml
```

More documentation for getting hardware running (ros topics, hardware prequisites, etc.) will be added in the future.


## Citation
```
@article{fan2019bayesian,
  title={Bayesian Learning-Based Adaptive Control for Safety Critical Systems},
  author={Fan, David D and Nguyen, Jennifer and Thakker, Rohan and Alatur, Nikhilesh and Agha-mohammadi, Ali-akbar and Theodorou, Evangelos A},
  journal={arXiv preprint arXiv:1910.02325},
  year={2019}
}
```

## Licensing
The code in this project is licensed under GNU General Public license v3.0.

