# **Extended Kalman Filter** 

---

**Extended Kalman Filter Project**

The goals / steps of this project are the following:
* Code must compile without errors with cmake and make
* px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52] for the Dataset 1.


[//]: # (Image References)

[image1]: ./result.png "Result"

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files

My project includes all the required cpp files.

#### 2. Submission includes functional code

The code is compileable with cmake and make in Linux environment.

#### 3. Submission code is usable and readable

The codes provide some comments and citation from the online course for a specific implementation.

### Model Architecture and Training Strategy

#### 1. Implementation Approach

Until now, this project is the simplest project that I have already done for this Nanodegree because in this project I only have to implement an existing algorithm with a software architecture that is already provided by Udacity.
In this writeup I will only tell my approach to achieve the required accuracy. My principal of coding is to make as minimum change as possible to an existing code. The reason is that I want to minimize the compatibility failure
with other components that I didn't touch after the implementation. For this reason, I tried to understand the architecture of the starter code from Udacity and then tried to do the implementation based on the current structure
without changing the function declaration. The implementations that I did are as follows:

1. Initialize all reusable matrices in the constructor to avoid memory allocation by each function call
2. Implement calculate jacobian
3. Implement cartesian to polar coordinate conversion
4. Normalizing angle - this is very important - after the coordinate conversion and also after the difference calculation between measured values and predicted values. Failure in this step can increase the rmse (error) value.

Here is the result after all of the implementations:

![alt text][image1]