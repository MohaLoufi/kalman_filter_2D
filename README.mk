# **2D Kalman Filter Example**

This repository contains a Python implementation of a 2D Kalman Filter, demonstrating how this powerful algorithm can be used to estimate the state of a system from noisy measurements. The code is based on the concepts explained in the excellent blog post ["How a Kalman filter works, in pictures"](https://bzarg.com/p/how-a-kalman-filter-works-in-pictures/) by Bzarg.

## **What is a Kalman Filter?**

A Kalman filter is an optimal estimation algorithm that uses a series of measurements observed over time, containing noise and other inaccuracies, and produces estimates of unknown variables that tend to be more precise than those based on a single measurement alone. It does this by predicting the next state of the system and then updating this prediction based on the new measurement.

The core idea, as illustrated in the linked blog post, is to combine two sources of information:

1. **Prediction:** Based on the system's model and the previous state estimate. This prediction has some uncertainty.  
2. **Measurement:** A new observation of the system, which also has its own uncertainty.

The Kalman filter optimally weighs these two sources of information to produce a new, more accurate state estimate with reduced uncertainty.

## **Project Overview**

This project implements a 2D Kalman filter for tracking an object moving with a constant velocity in a 2D plane. The system's state is defined by its position and velocity in both the x and y directions:

x=\[x y vx​ vy​​\]  
The code simulates a true trajectory and generates noisy measurements of the object's position. The Kalman filter then processes these noisy measurements to produce a smoothed estimate of the object's position and velocity.

## **Files**

* kalman\_filter\_2d.py: Contains the Python code for the 2D Kalman Filter implementation, including functions for simulating data and running the filter.  
* README.md: This file, providing an overview of the project and instructions.

## **How to Run**

1. **Prerequisites:** Make sure you have Python installed, along with the numpy and matplotlib libraries. You can install them using pip:  
   pip install numpy matplotlib

2. **Save the code:** Save the content of the kalman\_filter\_2d.py immersive document as a file named kalman\_filter\_2d.py in a local directory.  
3. **Run the script:** Open your terminal or command prompt, navigate to the directory where you saved the file, and run the script:  
   python kalman\_filter\_2d.py

This will execute the simulation and the Kalman filter, displaying two plots. To include these plots in this README, you will need to save them as image files (e.g., PNG) and then add the corresponding Markdown image links.

### **Expected Plots**

When you run the script, you will see the following plots:

* **2D Position Estimation Plot:** This plot shows the trajectory of the true position, the noisy measurements, and the Kalman filter's estimated position in the 2D plane. You will see how the filter's estimate smooths out the noisy measurements and tracks the true path.  
* **Kalman Gain and Uncertainty Plots:** This figure contains two subplots. The top subplot shows the Kalman Gain over time, indicating how much the filter relies on the new measurement versus its prediction. The bottom subplot shows the trace of the state covariance matrix (a measure of overall uncertainty) over time, demonstrating how the filter's uncertainty in its estimate changes as it incorporates more measurements.

**To include the actual plots:**

After running the kalman\_filter\_2d.py script and the plots are displayed by matplotlib, you can save them as image files. In the plot window, you can usually find a save icon (often a floppy disk). Save the 2D position plot as position\_plot.png and the Kalman Gain/Uncertainty plot as gain\_uncertainty\_plot.png (or similar names). Place these image files in your repository, and update the path/to/your/ part in the Markdown links above to the correct path relative to your README.md file.

## **Automotive Applications**

Kalman filters, like the 2D example provided here, are widely used in the automotive industry for various critical tasks, especially in the context of Advanced Driver-Assistance Systems (ADAS) and autonomous driving. Some key applications include:

* **Vehicle Localization and Tracking:** Fusing data from sensors such as GPS, IMUs, radar, and lidar to get a highly accurate and robust estimate of the vehicle's own position, velocity, and orientation, even in environments where GPS signals are weak or unavailable.  
* **Object Tracking:** Estimating the state (position, velocity, acceleration) and predicting the future trajectory of other vehicles, pedestrians, cyclists, and obstacles detected by sensors. This is fundamental for tasks like adaptive cruise control, collision avoidance, and automated driving decisions.  
* **Sensor Fusion:** Combining data from multiple disparate sensors to leverage their individual strengths and mitigate their weaknesses, resulting in a more reliable and comprehensive understanding of the vehicle's surroundings and its own state.  
* **Vehicle State Estimation:** Estimating internal vehicle states that are not directly measured, such as tire slip, vehicle sideslip angle, and roll/pitch angles, by integrating data from various chassis sensors.

This 2D Kalman filter example provides a foundational understanding of the core principles behind the state estimation techniques used in these advanced automotive systems.

## **Extending the Code**

This example uses a simple constant velocity model. You can extend this code to:

* Implement different system models (e.g., constant acceleration).  
* Incorporate different types of measurements.  
* Apply the filter to real-world data.

Feel free to explore and modify the parameters within the kalman\_filter\_2d.py file to see how they affect the filter's performance.
