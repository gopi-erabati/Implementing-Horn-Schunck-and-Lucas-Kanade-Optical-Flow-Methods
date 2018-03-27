# Implementing-Horn-Schunck-and-Lucas-Kanade-Optical-Flow-Methods

Optical flow is the pattern of apparent motion of objects, surfaces, and edges in a visual scene caused by the relative motion between an observer and a scene. It depends on Brightness Constancy assumption. 
Horn and Schunck is a global method to find optical flow. The main idea is to get optical flow smooth. As the Optical Flow Constraint Equation (OFCE) is an under constraint equation which cannot be solved for each pixel, they proposed to add other smoothness term to OFCE. This method works for small motion. 
Lucas Kanade is a local method. The main idea is the optical flow is constant on the neighborhood of the current point (x,y). Each neighbor gives one equation. Here we assume that pixel's neighbor have same velocity (u,v). by taking pixels in a neighborhood, we get an over determined system which can be solved by Linear least squares or by pseudo inverse.
