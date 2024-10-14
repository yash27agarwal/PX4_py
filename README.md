# PX4_py

**Author:** Agarwal Yash

## Introduction
This repository implements a PX4 control algorithm in Python. Please note that the repository is currently under development, and certain parts of the code may contain errors or incomplete features. As of now, the inner loop control and motor dynamics have been implemented, while the outer loop control is still in progress.

## Usage
To get started with the repository, follow these steps:

1. **Fork the Repository:** It is recommended to fork this repository to maintain your own copy and contribute to its development.
  
2. **Configure the `main.py` File:**
   - The inner loop control in `src/main.py` requires four key input variables:
     1. **Current Attitude (quaternions):** `att_q`
     2. **Attitude Setpoint (quaternions):** `att_setpoint_q`
     3. **Yaw Rate Setpoint:** `yawrate_d`
     4. **Thrust Setpoint:** `thrust`
   - You will need to update these values in `src/main.py` as per your specific use case and also need close the control loop.

3. **Run the Code:** Once the input values have been updated, you can execute the control algorithm. Ensure you are using the correct environment and dependencies.

To run the code, you need to run:
```
cd src
python3 -m main.py
```


## Parameters
All controller parameters are defined in the configuration file located at `config/config.yaml`. You can adjust the parameters to fine-tune the behavior of the inner loop control and motor dynamics.

## Development Status
- **Inner Loop Control:** Implemented
- **Motor Dynamics:** Implemented
- **Outer Loop Control:** Under development

If you encounter issues or have suggestions, feel free to raise an issue.

 