# DOS: A driving simulation benchmark DriveOcclusionSim consisting of diverse occlusion events
![intro](assets/intro.jpg)
> [Hao Shao](http://hao-shao.com/), [Letian Wang](https://letianwang0.wixsite.com/myhome), RuoBing Chen, [Steven L. Waslander](https://www.trailab.utias.utoronto.ca/), [Hongsheng Li](http://www.ee.cuhk.edu.hk/~hsli/), [Yu Liu](https://liuyu.us/)
> 
> [CVPR 2023](https://arxiv.org/abs/2305.10507)


This repository contains the benchmark proposed by the paper [ReasonNet: End-to-End Driving with Temporal and Global Reasoning
](https://arxiv.org/abs/2305.10507). 

This is a CARLA based framework providing diverse driving scenarios with occluded objects. The proposed DOS benchmark includes four types of challenging occlusion driving scenarios. Each of the four scenarios in the DOS benchmark comprises 25 different cases varying in the road environment and background traffic:
1. **Parked Cars**: The ego vehicle is driving in a straight lane with parked cars on the side. Pedestrians can first appear on the sidewalk (visible) and then suddenly emerge through the occluded areas between parked cars (occluded).
2. **Sudden Brake**: The ego vehicle is driving in a straight lane along with other vehicles ahead. Pedestrians can suddenly emerge from the sidewalks, causing the other vehicles to brake while remaining invisible to the ego vehicle.
3. **Left Turn**: The ego vehicle intends to perform an unprotected left turn at an intersection, but a truck in the opposite lane blocks the view of oncoming traffic, intermittently obscuring vehicles driving straight through the intersection.
4. **Red Light Infraction**: The ego vehicle is crossing an intersection after some trucks. A left-to-right vehicle running a red light suddenly appears, forcing the trucks to brake promptly. But the ego vehicle’s view toward the running-light vehicle is blocked by the trucks, so it remains invisible to the ego vehicle.

Compared to a previous occlusion benchmark [AUTOCASTSIM](https://github.com/hangqiu/AutoCastSim), the DOS benchmark: 1) includes occlusions of both vehicles and pedestrians, instead of only vehicles; 2) includes 100 cases of 4 scenarios, instead of only 3 cases of 3 scenarios; 3) considers specific occlusions that can potentially be resolved by temporal reasoning (intermittent occlusion, #1, #3) and global reasoning (constant occlusion but with interaction clues, #2, #4) about the scene, instead of random occlusions as in AUTOCASTSIM. Thus our scenarios can also serve as a good tracking-with-intermittent-occlusion benchmark and a People-as-Sensor benchmark.



## Prerequisites
Carla version: 0.9.10.1

If you haven't installed the environment of `leaderboard` or `scenario_runner` before:
```bash
cd /path/to/DOS
pip install -r leaderboard/requirements.txt
pip install -r scenario_runner/requirements.txt
```

## Setup

To use this benchmark for data collection or agent evaluation based on an existing project (like [InterFuser](https://github.com/opendilab/InterFuser), [LAV](https://github.com/dotchen/LAV)), you only need to follow these four steps:
1. Clone the repository using Git.
2. Go to your project directory and make a backup of your leaderboard and scenario_runner folders.
3. Move the DOS_benchmark folder to your project directory, and replace the existing leaderboard and scenario_runner folders with the ones provided in this repository.
4. Update leaderboard/scripts/run_evaluation.sh by setting ROUTES to DOS_benchmark/DOS_0X_town05.xml, SCENARIOS to DOS_benchmark/DOS_0X_town05.json (where X is the scenario id), and ensuring that TEAM_CONFIG, TEAM_AGENT, and CARLA_ROOT are correctly configured.


## Demo videos

Parked Cars(#1):


https://github.com/opendilab/DOS/assets/17512647/5df23b85-471f-4c72-9953-57fc346fc33e


Sudden Brak(#2):


https://github.com/opendilab/DOS/assets/17512647/f64e59a1-b9ef-4d53-b36d-9c9c36256f29


Left Turn(#3):


https://github.com/opendilab/DOS/assets/17512647/325855be-4285-4dda-b37d-09ee361da9c3


Red Light Infraction(#4):


https://github.com/opendilab/DOS/assets/17512647/69b26a9a-cff9-4b1e-881c-71e9b46cfe16



## Acknowledgements
This implementation is based on code from several repositories.
- [2020_CARLA_challenge](https://github.com/bradyz/2020_CARLA_challenge)
- [CARLA Leaderboard](https://github.com/carla-simulator/leaderboard)
- [Scenario Runner](https://github.com/carla-simulator/scenario_runner)

## Citation
If you find our repo or paper useful, please cite us as
```bibtex
@inproceedings{shao2023reasonnet,
  title={ReasonNet: End-to-End Driving with Temporal and Global Reasoning},
  author={Shao, Hao and Wang, Letian and Chen, Ruobing and Waslander, Steven L and Li, Hongsheng and Liu, Yu},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={13723--13733},
  year={2023}
}
```

## License
All code within this repository is under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
