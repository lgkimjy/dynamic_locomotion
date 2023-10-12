# Dynamic Locomotion

Robot Walking in desired velocity (optimal DCM + task-prioritized + KinWBC + DynWBC)

---

### TODO
- [x] Nominal Gait Planning (DCM *or* Raibert Heuristic)
  - [ ] Swing Foot Trajectory Generator
  - [ ] CoM Trajectory Generator
  - [ ] QP-based Swing Foot Trajectory Generation (QPOASES)
- [ ] Reaction Force Deployment
- [ ] KinWBC
  - [ ] Task-Prioritized ( 4 tasks )
- [ ] DynWBC
  - [ ] QP formulation (quadprogg)
  - [ ] find $\delta_{\ddot{q}}$ and $\delta_{f}$
- [ ] Joint Level Controller

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
