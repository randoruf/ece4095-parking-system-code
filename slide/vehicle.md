

# Vehicle Model

### Vehicle Dimension 

https://au.mathworks.com/help/driving/ref/sedan.html

```
self.width = 2.20        			# [m] width of vehicle
self.length = 4.82					# [m] length of vehicle 

self.rf_distance = 3.80    			# [m] distance from rear axle to front end of vehicle
self.rb_length = 1.12    			# [m] distance from rear axle to back end of vehicle
self.rear_front_axle_distance = 2.8     # [m] distance from rear axle to front axle
```

<img src="image-20210307200520861.png" alt="image-20210307200520861" style="zoom:50%;" />

<img src="image-20210307200534296.png" alt="image-20210307200534296" style="zoom:50%;" />



---

### Kinematic Bicycle Model 

- [Lesson 1: Trajectory Propagation - Module 6: Reactive Planning in Static Environments | Coursera](https://www.coursera.org/lecture/motion-planning-self-driving-cars/lesson-1-trajectory-propagation-5hguf)
- [RRT Page: Photo and Animation Gallery (lavalle.pl)](http://lavalle.pl/rrt/gallery_carsmooth.html)
- [ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp Source File (kavrakilab.org)](https://ompl.kavrakilab.org/RigidBodyPlanningWithIntegrationAndControls_8cpp_source.html)



![image-20210316183801485](image-20210316183801485.png)

- Note that we have a maximum **steering angle** and maximum **steering rate**. 

![image-20210316184414608](image-20210316184414608.png)

1. The mechanical limit on the steering angle <img src="image-20210316184556539.png" alt="image-20210316184556539" style="zoom:67%;" />
2. The limit on the steering rate <img src="image-20210316184607586.png" alt="image-20210316184607586" style="zoom: 67%;" />
3. The limit on the car speed <img src="image-20210316184703612.png" alt="image-20210316184703612" style="zoom: 67%;" />

Note that the ***control inputs*** now are $$v_{\delta}$$ and $$v_{r}$$ .

