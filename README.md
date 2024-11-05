# Preliminary Installations

**RecoNIC/xsim Requirements**
- Vivado 2021.2 (Enterprise Version)
- ERNIC, Vitis Networking P4 License
- python >= 3.8

**SimBricks Requirements**
- docker

**Notes**:
- The software/IP licenses can be purchased or applied through AMD University Program. Alternatively, the evaluation license can be used to temporarily evaluate this project.
- The elboration/simulation of RecoNIC using Vivado requires sufficient system memory. Insufficient memory may lead to degraded performance due to swap files, or fatal errors. *Each* xsim simulation may peak to ~10GB of memory usage.
- If `TimeoutError` occurs, try increasing the timeout duration in `simbricks/orchestration/exectools.py` as the elaboration process may take awhile depending on CPU and Memory availalbe.


# RecoNIC Experiment

## rdma_2write experiment
Execute `python3 run.py --verbose --force pyexps/reconic_write.py` to run the experiment. The output can be found in the `/out` folder.