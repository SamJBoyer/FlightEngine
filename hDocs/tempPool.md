<wants>

1. Realistic conservation of energy: flying upwards will cause an increase in altitude but can decrease speed, depending on engine thrust / weight
2. Novel force vectors that act on the plane's body. Vectors can cause translation and rotation of the plane's body. Useful to model the thrust of engines and also random impacts
3. Plane to be treated as a single rigid body object with flight properties that define its aerodynamic performance
4. The VPC should be able to compensate for unbalanced forces, like unbalanced engines.
5. Gravity to act on the body of the plane
6. A novel point should be able to be fed into the VPC and the VPC should be able to direct the plane to fly towards that point.
7. Compression: flying at too high of a speed causes control surfaces to stiffen and reduce control effectiveness
8. The flight performance depends on the speed of the plane. At certain speeds the plane should stall and become unresponsive
9. The plane should be able to roll, pitch, and yaw using alieron, elevators, and rudder control
10. Speed should be measured in km/h, and altitude in meters. Metric is the standard unit


11. Clear designations of what gets passed accross the boundary
12. Should be able to run as a back-end that The flight logic on a front end
13. Flight engine that produces psuedo-realistic features
14. Plane can be assumed to weight 8000kg
15. Plane should have a verticle loop time of 15 seconds starting at 400km/h
16. Plane should have a flat circle time of 15 seconds starting at 400km/h and maintain a roughtly 400km/h speed throughout the turn
17. Plane should be able to complete a full roll in 6 seconds
18. Plane should have a stall speed of 150 km/h when flying at 0 degrees, and a stall speed of 40km at 90 degrees pointing upwards
19. Plane should be able to maintain a 20 degree climb at 280km/h

</wants>
<dontWants>

1. Logic regarding landing, wind, or weather
2. Logic regarding fuel, unstable, liquids or shifting weight
3. Complicated physics regarding engines. Engines can be modeled as simple thrust vectors.
4. Logic regarding damage or lost compartments
5. Collisions, collision hitboxes, Colliders.

</dontWants>
<questions>

- Can the VPC be seperated from the physics engine, or is knowledge of the physics engine required for adjustments. 
- big missing question about how to actually implement the tests 

</questions>
<wonders>

Initialized as empty.

</wonders>
