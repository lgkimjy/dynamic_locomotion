# Dynamic Locomotion

Robot Walking in desired velocity (optimal DCM + task-prioritized + KinWBC + DynWBC)

---

### TODO
- [x] Modulize the Classes
- [x] Nominal Gait Planning (DCM *or* Raibert Heuristic)
  - [x] Swing Foot Trajectory Generator
  - [x] CoM Trajectory Generator
  - [x] QP-based Swing Foot Trajectory Generation (eigen-quadprogpp)
- [x] Reaction Force Deployment
  - [x] Centroidal Dyanmics
  - [x] Solve using QP, in every contact states 
- [x] KinWBC
  - [x] Task-Prioritized ( 4 tasks )
  - [x] kinematics verification check
- [x] DynWBC
  - [x] QP formulation (eigen-quadprogg)
  - [x] find $\delta_{\ddot{q}}$ and $\delta_{f}$
- [x] Joint Level Controller
- [ ] visualize data, logger

---

### Reference
```
@inproceedings{khadiv2016step,
  title={Step timing adjustment: A step toward generating robust gaits},
  author={Khadiv, Majid and Herzog, Alexander and Moosavian, S Ali A and Righetti, Ludovic},
  booktitle={2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids)},
  pages={35--42},
  year={2016},
  organization={IEEE}
}
```
```
@article{kim2020dynamic,
  title={Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control},
  author={Kim, Donghyun and Jorgensen, Steven Jens and Lee, Jaemin and Ahn, Junhyeok and Luo, Jianwen and Sentis, Luis},
  journal={The International Journal of Robotics Research},
  volume={39},
  number={8},
  pages={936--956},
  year={2020},
  publisher={SAGE Publications Sage UK: London, England}
}
```
